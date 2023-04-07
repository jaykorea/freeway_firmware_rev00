/*
 * mainpp.cpp
 *
 *  Created on: Sep 16, 2022
 *      Author: van
 */

#include <mainpp.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
//#include <ros/service_client.h>
#include <freeway_joyfw/stm_fw_msg.h>
#include <freeway_joyfw/stm_am_msg.h>
//#include <freeway_joyfw/stm_fw_srv.h>
#include <sensor_msgs/Range.h>
#include <freeway_joyfw/stm_fw_sonar_msg.h>

// Node:
ros::NodeHandle nh;
//using freeway_joyfw::stm_fw_srv;
// Publisher:
geometry_msgs::Twist cmd_vel_msg;
freeway_joyfw::stm_fw_msg stm_pub_msg;
ros::Publisher freeway_diagnostics("freeway/diagnostics", &stm_pub_msg);

freeway_joyfw::stm_fw_sonar_msg range_msg;
sensor_msgs::Range range_left;
sensor_msgs::Range range_right;
ros::Publisher pub_range("freeway/ultrasound", &range_msg);

// Service Client:
//ros::ServiceClient<Stm32_Status::stm_status::Request, Stm32_Status::stm_status::Response>clt("/e_stop_status");
//Stm32_Status::stm_status::Request ESTOP_STATUS;
//Stm32_Status::stm_status::Response RESULT;

//void auto_mode_cb(const stm_fw_srv::Request &req, stm_fw_srv::Response &res){
//	bool tf = false;
//	am_status = &tf;
//	//HAL_Delay(5);
//}
//
//void manual_mode_cb(const stm_fw_srv::Request &req, stm_fw_srv::Response &res){
//	bool tf = true;
//	am_status = &tf;
//	//HAL_Delay(5);
//}

// Service Server
//ros::ServiceServer<stm_fw_srv::Request, stm_fw_srv::Response> server("auto_mode", &auto_mode_cb);
//ros::ServiceServer<stm_fw_srv::Request, stm_fw_srv::Response> server2("manual_mode", &manual_mode_cb);

bool g_am_status = false;
bool *e_stop_status, *am_status;
bool pin_stat = false;

// Subscriber:
void am_status_cb(const freeway_joyfw::stm_am_msg &msg) {
	g_am_status = msg.am_status2;
	am_status = &g_am_status;
	HAL_Delay(1);
}
ros::Subscriber<freeway_joyfw::stm_am_msg> am_status_sub("freeway/am_status", &am_status_cb);

long map(uint32_t a, long b, long c, long d, long e) {
	return (a - b)*(e - d)/(c - b) + d;
}

uint32_t previous_time;
uint32_t pub_period_time = 100;

// Setup node:
void setup(void) {
  bool init_pin_stat = false;
  nh.initNode();
//  nh.advertiseService(server);
//  nh.advertiseService(server2);
  nh.subscribe(am_status_sub);
  nh.advertise(freeway_diagnostics);
  nh.advertise(pub_range);
  range_msg.range_left.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.range_right.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.range_left.field_of_view = 0.26;
  range_msg.range_right.field_of_view = 0.26;
  range_msg.range_left.min_range = 0.03;
  range_msg.range_right.min_range = 0.03;
  range_msg.range_left.max_range = 4.0;
  range_msg.range_right.max_range = 4.0;

  //nh.advertiseService(server2);
  //*e_stop_status = true;
  am_status = &g_am_status;

  init_pin_stat = HAL_GPIO_ReadPin(e_stop_GPIO_Port, e_stop_Pin);
  if(init_pin_stat == false){ //estop on
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);

  }
  else { //estop off
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
  }
}

// Loop:
uint8_t loop(uint32_t x_val, uint32_t y_val, double r_val, double l_val) {
	static uint8_t r_led_mode = 0;
	double axis_X, axis_Y;
	//bool axis_B = true;
	double l_scale_ = 0.3;
	double a_scale_ = 0.4;


	range_msg.range_left.range   = l_val;
	range_msg.range_right.range  = r_val;

	pin_stat = HAL_GPIO_ReadPin(e_stop_GPIO_Port, e_stop_Pin);
	e_stop_status = &pin_stat;

	if(previous_time + pub_period_time <= HAL_GetTick()) {
	  if (*e_stop_status==true && *am_status==true) { //if var 'am_status == true' , it defines manual mode
		  axis_X = map(x_val,0,4095,-10000,10000) / (float)10000.0;
		  axis_Y = map(y_val,0,4095,-10000,10000) / (float)10000.0;

		  //cmd_vel_msg.linear.x = l_scale_ * axis_X;
		  //cmd_vel_msg.angular.z = a_scale_ * axis_Y;
		  stm_pub_msg.am_status = true;
		  stm_pub_msg.e_stop_status = true;
		  stm_pub_msg.cmd_vel_mcu.linear.x = l_scale_ * axis_X;
		  stm_pub_msg.cmd_vel_mcu.angular.z = -a_scale_ * axis_Y;

		  freeway_diagnostics.publish(&stm_pub_msg);
		  r_led_mode = 2;

		  //HAL_UART_Transmit_IT(&huart3, stm_pub_msg, sizeof(stm_pub_msg));
	  }
	  else if (*e_stop_status==false) // e_stop is on & *am_status is on/off
	  {
		  stm_pub_msg.am_status = *am_status;
		  stm_pub_msg.e_stop_status = false;
		  stm_pub_msg.cmd_vel_mcu.linear.x = 0;
		  stm_pub_msg.cmd_vel_mcu.angular.z = 0;

		  freeway_diagnostics.publish(&stm_pub_msg);

		  r_led_mode = 1;

		  //HAL_UART_Transmit_IT(&huart3, stm_pub_msg, sizeof(stm_pub_msg));
	  }
	  else r_led_mode = 0;

	  pub_range.publish(&range_msg);
	  previous_time = HAL_GetTick();
	}

	  nh.spinOnce();

	  return r_led_mode;
	}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
  for (int i=0; i<1; i++){
	pin_stat = HAL_GPIO_ReadPin(e_stop_GPIO_Port, e_stop_Pin);
  }
  e_stop_status = &pin_stat;
//  if(pin_stat == false) {
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
//  }
//  else {
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
//	  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
//  }


  /*
  if(*e_stop_status==false) {
	  tongsin[0] = 0;
	  tongsin[1] = 0;
  }
  */
//  bool pin_stat3 = HAL_GPIO_ReadPin (GPIOB, GPIO_Pin);
//  if(e_stop_status == true && axis_B == true) {
//	  cmd_vel_msg.linear.x = 0;
//	  cmd_vel_msg.angular.z = 0;
//
//	  cmd_vel_pub.publish(&cmd_vel_msg);
//  }
//  else if(e_stop_status == true && axis_B == false) {
//	  cmd_vel_msg.linear.x = 0;
//	  cmd_vel_msg.angular.z = 0;
//
//	  cmd_vel_pub.publish(&cmd_vel_msg);
//  }
//  else if(e_stop_status == false && axis_B == false) {
//	  cmd_vel_msg.linear.x = l_scale_ * axis_X;
//	  cmd_vel_msg.angular.z = a_scale_ * axis_Y;
//
//	  cmd_vel_pub.publish(&cmd_vel_msg);
//  }
//  ESTOP_STATUS.estop_query = e_stop_status;
//  ESTOP_STATUS.am_query = true;
//
//  RESULT.result = 1;
//
//  clt.call(ESTOP_STATUS, RESULT);
  //HAL_Delay(1);

}

//void HAL_GPIO_EXTI_Callback3(uint16_t GPIO_Pin)
//{
//  /* Prevent unused argument(s) compilation warning */
//  UNUSED(GPIO_Pin);
//  static bool pin_stat = true;
//  /* NOTE: This function Should not be modified, when the callback is needed,
//           the HAL_GPIO_EXTI_Callback could be implemented in the user file
//   */
//  for (int i=0; i<10; i++){
//	pin_stat = HAL_GPIO_ReadPin (GPIOB, GPIO_Pin);
//  }
//  //am_status = &pin_stat;
//  //HAL_Delay(1);
//}
