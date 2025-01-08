#pragma once

#include <cassert>
#include <cstring>

#define BOARD_ESP32CAM

#define UART_RX_BUFFER_SIZE_MSP_OSD      512
#define UART_TX_BUFFER_SIZE_MSP_OSD      256

#define UART_RX_BUFFER_SIZE_MAVLINK      512
#define UART_TX_BUFFER_SIZE_MAVLINK      256

//===============================================================
//For esp32cam
#ifdef BOARD_ESP32CAM


//---- CONFIG 1 -----
//   Debug log is on UART0
//   UART2:  MSP-OSD RX=13 TX=12
//   REC BUTTON: 3 (RX0)
//   STATUS LED1: 33 (existing LED)
//   STATUS LED2: 4 (existing FLASH LED)

#define UART_MSP_OSD UART_NUM_2
#define UART2_RX_BUFFER_SIZE UART_RX_BUFFER_SIZE_MSP_OSD
#define UART2_TX_BUFFER_SIZE UART_TX_BUFFER_SIZE_MSP_OSD

#define CAMERA_MODEL_AI_THINKER
//#define DVR_SUPPORT

#define INIT_UART_0
#define TXD0_PIN    1
#define RXD0_PIN    4  //moved from pin 3 to pin 4 to free pin 3 for a REC button

#define INIT_UART_2
#define TXD2_PIN    12   //should be low at boot!!!
#define RXD2_PIN    13 
#define UART2_BAUDRATE 115200

#define STATUS_LED_PIN GPIO_NUM_33
#define STATUS_LED_ON 0
#define STATUS_LED_OFF 1
#define FLASH_LED_PIN GPIO_NUM_4
#define REC_BUTTON_PIN  GPIO_NUM_3  //RX0
//----------------------


/*
//---- CONFIG 2 -----
//   Debug log on pin 33 (exiting LED)
//   UART0:  Mavlink RX=3 TX=1 
//   UART2:  MSP-OSD RX=13 TX=12
//   NO REC BUTTON
//   STATUS LED: 4 (existing FLASH LED)

#define UART_MAVLINK UART_NUM_1
#define UART1_RX_BUFFER_SIZE UART_RX_BUFFER_SIZE_MAVLINK
#define UART1_TX_BUFFER_SIZE UART_TX_BUFFER_SIZE_MAVLINK

#define UART_MSP_OSD UART_NUM_2
#define UART2_RX_BUFFER_SIZE UART_RX_BUFFER_SIZE_MSP_OSD
#define UART2_TX_BUFFER_SIZE UART_TX_BUFFER_SIZE_MSP_OSD

#define CAMERA_MODEL_AI_THINKER
#define DVR_SUPPORT

#define INIT_UART_0
#define TXD0_PIN    33
#define RXD0_PIN    33
#define UART0_BAUDRATE 115200

#define INIT_UART_1
#define TXD1_PIN    1
#define RXD1_PIN    3
#define UART1_BAUDRATE 115200

#define INIT_UART_2
#define TXD2_PIN    12   //should be low at boot!!!
#define RXD2_PIN    13 
#define UART2_BAUDRATE 115200

#define FLASH_LED_PIN GPIO_NUM_4
//----------------------
*/

#endif

//===============================================================
//===============================================================
//for XIAO ESP32S3 Sense
//https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_SCH_v1.1.pdf
//https://files.seeedstudio.com/wiki/SeeedStudio-XIAO-ESP32S3/res/XIAO_ESP32S3_ExpBoard_v1.0_SCH.pdf

#ifdef BOARD_XIAOS3SENSE

//---- CONFIG 1 -----
//  Debug is on USB UART
//  UART1:  MSP-OSD  RX=D3 TX=D1
//  UART2:  MAVLINK  RX=D6 TX=D&
//  REC BUTTON: GPIO0  (existing flash button)
//  STATUS LED: GPIO1

//define to use DisplayPort OSD on UART1
#define UART_MSP_OSD UART_NUM_1
#define UART1_RX_BUFFER_SIZE UART_RX_BUFFER_SIZE_MSP_OSD
#define UART1_TX_BUFFER_SIZE UART_TX_BUFFER_SIZE_MSP_OSD

//define to use mavlink telemetry on UART2 
#define UART_MAVLINK UART_NUM_2
#define UART2_RX_BUFFER_SIZE UART_RX_BUFFER_SIZE_MAVLINK
#define UART2_TX_BUFFER_SIZE UART_TX_BUFFER_SIZE_MAVLINK

#define CAMERA_MODEL_XIAO_ESP32S3
#define DVR_SUPPORT
#define STATUS_LED_PIN GPIO_NUM_1
#define STATUS_LED_ON 1
#define STATUS_LED_OFF 0
#define REC_BUTTON_PIN  GPIO_NUM_0

#define INIT_UART_1
#define TXD1_PIN    GPIO_NUM_2 //D1
#define RXD1_PIN    GPIO_NUM_4 //D3
#define UART1_BAUDRATE 115200

#define INIT_UART_2
#define TXD2_PIN    GPIO_NUM_43 //D6
#define RXD2_PIN    GPIO_NUM_44 //D7
#define UART2_BAUDRATE 115200
//----------------------

#endif
//===============================================================

//write raw MJPEG stream instead of avi
//#define WRITE_RAW_MJPEG_STREAM

//#define TEST_AVI_FRAMES

//===============================================================

#define MAX_SD_WRITE_SPEED_ESP32   (800*1024) //esp32 can hadle 1.9mb writes, but in this project it's 0.8mb max due to overal system load (otherwise we miss camera data callback)
#define MAX_SD_WRITE_SPEED_ESP32S3 (1800*1024) //can  write 1900 but we set to 1800 due to overal system load



////////////////////////////////////////////////////////////////////////////////////


//extern bool isHQDVRMode();

























#include <Arduino.h>
#include <nvs_flash.h>
#include <nvs.h>

//#include "wifi.h"

//#include "config.h"



// class FPV_Cam_Config {
// public:

//     struct WiFi_Config{
//         wifi_mode_t wifi_mode = WIFI_MODE_AP;
//         wifi_phy_rate_t wifi_rate = WIFI_PHY_RATE_11M_L;
//         uint8_t wifi_channel = 13;
//     } wifi_config;
    
//     struct Pin_Config{

//     } pin_config;

//     struct FEC_Config{

//     } fec_config;

// } fpv_cam_config_class;
