/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "rtos.h"
#include "PwmIn.h"
using namespace std::chrono;

////////////////ROS////////////////////
#include <ros.h>
#include <std_msgs/Empty.h>

#include <stdint.h>

#include <geometry_msgs/Point.h>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
//#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <tf/tf.h>
//#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Joy.h>
//////////////////////////////////////////
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
///////////////////////////////////////////
bool sendPacket(uint32_t id, uint8_t packet[], int32_t len);
void vesc_can_set_duty(uint8_t controller_id, float duty);
void comm_can_set_rpm(uint8_t controller_id, float rpm);

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string

typedef enum {
   CAN_PACKET_SET_DUTY = 0,
   CAN_PACKET_SET_CURRENT,
   CAN_PACKET_SET_CURRENT_BRAKE,
   CAN_PACKET_SET_RPM,
   CAN_PACKET_SET_POS,
   CAN_PACKET_FILL_RX_BUFFER,
   CAN_PACKET_FILL_RX_BUFFER_LONG,
   CAN_PACKET_PROCESS_RX_BUFFER, 
   CAN_PACKET_PROCESS_SHORT_BUFFER,
   CAN_PACKET_STATUS
} CAN_PACKET_ID;

CAN CAN0(PB_8, PB_9, 500000);

///////////////////CAN_BUS/////////////////////
CANMessage Txmsg;

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}

void buffer_append_int32ex(char* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1e3), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

void comm_can_set_duty(uint8_t controller_id, float duty)
{
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
   sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
   int32_t send_index = 0;
   uint8_t buffer[4];
   buffer_append_int32(buffer, (int32_t)rpm, &send_index);
   sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

//https://github.com/skipper762/teensy_VESC_CANBUS
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1e6), &send_index);
  sendPacket(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}


bool sendPacket(uint32_t id, uint8_t packet[], int32_t len)
{
   if (CAN0.write(CANMessage(id, (const char*)packet, sizeof(packet),CANData,CANExtended)))
   {
   
      return true;
   }
   else {
      //Serial.println("Error Sending Message...");
      return false;
   }
}
/////////////////////
//comm_can_set_rpm(71, 2435.4   ); //60*3*13.53     60rpm * 3* 13.53 =   ? erpm that equals to 60rpm
//////////////////

////////////////ROBOT PARAM/////////////////////////////
// chassis definition  
// https://github.com/supercrazysam/kinematics/blob/master/Wheeled%20Mobile%20Robot/Mecanum%20wheel/4%20wheel/pxc3901586.pdf
//          |Forward|
//     0   (71)                    1   (73)
//
//
//
//     2   (72)                    3    (74)
//          |Backward|
float linear_x  = 0;
float linear_y  = 0;
float angular_z = 0;

int motor1   = 0; 
int motor2   = 0;
int motor3   = 0; 
int motor4   = 0;

float pi = 3.14159265;

//Jean
//float lx = 0.27;   //54cm left to right   (wheels)
//float ly = 0.1935;  //38.7cm  front to back (wheels)

float lx = 0.22; //0.27;   //54cm left to right   (wheels)
float ly = 0.15; //0.1935;  //38.7cm  front to back (wheels)


float wheelR = 0.0762; // 6inch mecanum wheel   radius = 3inch  = 0.0762meter
float gear_ratio = 8.63;  //jean 13.53;
float pole_pair = 3.0;

float radian_to_erpm_convert = (1/((2*pi)/60)) * gear_ratio * pole_pair;
/////////////////////
void inverse(float vx, float vy, float w)
{
    motor1 = int( (1/wheelR) * (vx - vy - (lx + ly) * w ) * radian_to_erpm_convert );  //0
    motor2 = int( (1/wheelR) * (vx + vy + (lx + ly) * w ) * radian_to_erpm_convert );  //1
    motor3 = int( (1/wheelR) * (vx + vy - (lx + ly) * w ) * radian_to_erpm_convert );  //2
    motor4 = int( (1/wheelR) * (vx - vy + (lx + ly) * w ) * radian_to_erpm_convert );  //3
}

///////////////////SETUP NODEHANDLE and track last_time of important events///////////////
ros::NodeHandle nh;
//DigitalOut myled(LED1);    DISABLE since interfere

double last_control_msg_time = 0.0;           //last time that stm32 get control message
double last_rosspin_time = 0.0;               //last time that stm32 rospin once
double last_motor_can_control_time = 0.0;     //last time that stm32 send CAN bus control signal

double ros_cmd_vel_timeout = 1.0;             //stop if didnt get any cmd_ vel in the past N seconds


///////////////////////////////CALLBACKS/////////////////////
//void messageCb(const std_msgs::Empty& toggle_msg){
//    myled = !myled;   // blink the led
//    last_control_msg_time = nh.now().toSec();
//}

void cmdvelCb(const geometry_msgs::Twist& cmd_vel_msg){  
    linear_x  = cmd_vel_msg.linear.x;
    linear_y  = cmd_vel_msg.linear.y;
    angular_z = cmd_vel_msg.angular.z;

    last_control_msg_time = nh.now().toSec(); //cmd_vel_msg.header.stamp.toSec();
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/chassis_cmd_vel", &cmdvelCb);

//          |Forward|
//     0 motor1  (71)                    1  motor2 (73)
//
//
//
//     2 motor3  (72)                    3  motor4  (74)
//          |Backward|
void motor_update()
{
    comm_can_set_rpm(71,  -motor1); 
    wait_us(500); 
    comm_can_set_rpm(73,  motor2);
    wait_us(500); 

    comm_can_set_rpm(72,  -motor3);  
    wait_us(500); 
    comm_can_set_rpm(74,  motor4);
    wait_us(500); 

    last_motor_can_control_time = nh.now().toSec();
}

void ros_update()
{
    nh.spinOnce();
}

int main() {

    motor1  = 0; 
    motor2  = 0;
    motor3  = 0; 
    motor4  = 0;

    linear_x   = 0;
    linear_y   = 0;
    angular_z  = 0;


    nh.getHardware()->setBaud(115200);
    
    nh.setSpinTimeout(10); //25  //50     right now this is the only delay present in the loop, that could affect interrupt

    nh.initNode();
    //nh.subscribe(sub);
    nh.subscribe(cmd_vel_sub);


    //while (!nh.connected() )   {nh.spinOnce();} //added to make sure connects

    //wait(0.5);
    ThisThread::sleep_for(5s);
    while (true)
    {



        /////////////////////////////////////////////////////////////////////////////
        if ( (nh.now().toSec() - last_rosspin_time) > 0.025 ) //run at around 40hz    sync desired linear/agular speed from upstream ROS
        {
            nh.spinOnce();
            //myled = !myled;          //dont need this to check if the conenction is alive,  either computer will say error, or the toggle led wont turn on LED
            last_rosspin_time = nh.now().toSec();
        }
       ////////////////////////////////////////////////////////////////////////////////////
       if ( ( (nh.now().toSec() - last_control_msg_time) < ros_cmd_vel_timeout   ) )
        {
            inverse(linear_x,  linear_y, angular_z);

        } 
        else  //else timeout from rosserial uplink
        {
           motor1  = 0;
           motor2  = 0;
           motor3  = 0;
           motor4  = 0;
        }
        /////////////////////////////////////////////////
        if ( (nh.now().toSec() - last_motor_can_control_time) > 0.025 ) //run 40Hz motor control      maybe this wont occupy the cpu that much
        {
            motor_update();
            last_motor_can_control_time = nh.now().toSec();
        }
    }
}