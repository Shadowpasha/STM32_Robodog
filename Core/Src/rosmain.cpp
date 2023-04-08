#include "common.h"
#include "rosmain.h"
//#include "cmsis_os.h"
//#include "FreeRTOS.h"
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char hello[] = "Hello world!";

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){

  nh.getHardware()->flush();

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

  nh.getHardware()->reset_rbuf();

}

extern "C" void setup(void)
{
  nh.initNode();
  nh.advertise(chatter);
}

extern "C" void ChatterLoop(void)
{

  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);

  str_msg.data = hello;
  chatter.publish(&str_msg);
  nh.spinOnce();

}
