/*
 * stm32_ros.cpp
 *
 *  Created on: Nov 26, 2020
 *      Author: colson
 */



#include "stm32_ros.h"
#include "ros/time.h"
#include "ros.h"
#include "std_msgs/Byte.h"
#include "std_msgs/ByteMultiArray.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"


#define AX_WRITE_DATA  3


extern UART_HandleTypeDef huart1;

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher pub_chat("AuCAR/chatter", &str_msg);
char hello[] = "Hello world!";

void angle_cb(const geometry_msgs::Vector3& msg);

void ax12_SetRegister(uint8_t id, uint8_t regstart, uint16_t data);

uint8_t sendData[100] = { 0, };

uint8_t angle_cb_cnt;

ros::Subscriber<geometry_msgs::Vector3> angle_sub("gimbal/angles", &angle_cb);


float yaw_data, pitch_data, roll_data;

uint32_t nowTick, pastTick;

int __printf__io__putchar(int ch)
{
	uint8_t data = ch;

	//TODO change MAX485 or CAN line
	//__usart2.write(&data, 1);

	return ch;
}



void ros_init(void)
{

	nh.initNode();
	nh.advertise(pub_chat);
	nh.subscribe(angle_sub);

	nowTick = HAL_GetTick();
	pastTick = nowTick;
}

void ros_run(void)
{

	ax12_SetRegister(1, 30, 512-yaw_data);
	HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);
	ax12_SetRegister(1, 32, 256);
	HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);

	ax12_SetRegister(2, 30, 512+pitch_data);
	HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);
	ax12_SetRegister(2, 32, 256);
	HAL_UART_Transmit(&huart1, sendData, 9, 0xFF);


	nowTick = HAL_GetTick();
	if(nowTick - pastTick > 100)
	{
		str_msg.data = hello;
		pub_chat.publish(&str_msg);
		nh.spinOnce();
		pastTick = nowTick;
	}
}


void angle_cb(const geometry_msgs::Vector3& msg)
{
	roll_data = msg.x * 180 / 3.14;
	pitch_data = msg.y * 180 / 3.14;
	yaw_data = msg.z * 180 / 3.14;
}


void ax12_SetRegister(uint8_t id, uint8_t regstart, uint16_t data) {
	sendData[0] = 0xFF;
	sendData[1] = 0xFF;
	sendData[2] = id;
	sendData[3] = 5;
	sendData[4] = AX_WRITE_DATA;
	sendData[5] = regstart;
	sendData[6] = (data & 0xFF);
	sendData[7] = (data & 0xFF00) >> 8;
	sendData[8] = (0xFF
			- ((id + 5 + AX_WRITE_DATA + regstart + (data & 0xFF)
					+ ((data & 0xFF00) >> 8)) % 256));

}

