/*
 * stm32_ros.h
 *
 *  Created on: Nov 26, 2020
 *      Author: colson
 */

#ifndef STM32_ROS_H_
#define STM32_ROS_H_


#ifdef __cplusplus
extern "C" {
#endif


int __printf__io__putchar(int ch);

void ros_init(void);

void ros_run(void);

#ifdef __cplusplus
}
#endif
#endif /* STM32_ROS_H_ */
