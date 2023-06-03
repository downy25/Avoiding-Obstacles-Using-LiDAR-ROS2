/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _DXL_HPP_//_DXL_HPP 상수가 정의되어 있지 않으면
#define _DXL_HPP_//정의하라

#include <fcntl.h>//파일을 열고 다양한 작업을 하기위한 헤더파일
#include <termios.h>//터미널 I/O 인터페이스에서 사용하는 정의가 포함된 헤더파일
#include "dynamixel_sdk.h"              // Uses Dynamixel SDK library
#define STDIN_FILENO 0//기호상수 정의
#include <stdlib.h>//c언어 표준 라이브러리
#include <stdio.h>//c언어 표준 라이브러리

// Control table address for AX-12W and MX-12W
//AX-12W와  MX-12W의 Control table 주소
#define ADDR_MX_TORQUE_ENABLE           24                  
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32

// Control table address for XL and XC model
//XL 시리즈와  XC 시리즈의 Control table 주소
#define ADDR_XL_TORQUE_ENABLE           64                 
#define ADDR_XL_OPERATING_MODE          11                 
#define ADDR_XL_GOAL_POSITION           116
#define ADDR_XL_PRESENT_POSITION        132
#define ADDR_XL_GOAL_VELOCITY           104
#define ADDR_XL_PRESENT_VELOCITY        128

// Data Byte Length for AX-12W and MX-12W
//AX-12W와 MX-12W의 데이터 길이
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2

// Data Byte Length for XL and XC model
//XL,XC 시리즈의 데이터 길이
#define LEN_XL_GOAL_POSITION            4
#define LEN_XL_PRESENT_POSITION            4
#define LEN_XL_GOAL_VELOCITY               4
#define LEN_XL_PRESENT_VELOCITY            4

// Protocol version for AX-12W and MX-12W
#define PROTOCOL_VERSION                1.0//프로토콜 버전                  
// Protocol version for XL and XC model
//#define PROTOCOL_VERSION                2.0                 

// Default setting
#define DXL1_ID                         1                
// Dynamixel#1 ID: 1 //왼쪽바퀴
#define DXL2_ID                         2               
// Dynamixel#2 ID: 2 //오른쪽 바퀴
#define BAUDRATE                        2000000         
// for AX-12W and MX-12W model //MX,AX모델의 보드레이트 속도
//#define BAUDRATE                        4000000       
// for XL and XC model
#define DEVICENAME                      "/dev/ttyUSB0"  
// Check which port is being used on your controller //사용할 컨트롤러         
// ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
#define TORQUE_ENABLE                   1               
// Value for enabling the torque //토크활성화 값
#define TORQUE_DISABLE                  0               
// Value for disabling the torque //토크 비활성화 값
#define OPMODE_XL_VELOCITY              1               
#define OPMODE_XL_POSITION              3               
#define OPMODE_XL_PWM                    16              
#define ESC_ASCII_VALUE                 0x1b

class Dxl
{
private:
    //멤버변수 정의
    int port_num;
    int group_num;
    int dxl_comm_result;             // Communication result
    uint8_t dxl_addparam_result;     // AddParam result
    uint8_t dxl_error;               // Dynamixel error
    dynamixel::PortHandler* portHandler;
    dynamixel::PacketHandler* packetHandler;
public:
    //멤버 함수 선언
    Dxl(void);
    bool dxl_open(void);
    void dxl_close(void);
    bool dxl_set_velocity(int goal_velocity1, int goal_velocity2);
    void dxl_xl_open(void);
    void dxl_xl_close(void);
    //int dxl_xl_set_velocity(int goal_velocity1, int goal_velocity2);
    unsigned int vel_convert(int speed);
    void vel_convert_rpm(double linear_speed, double angular_speed, int* left_rpm, int* right_rpm);
    static int getch(void);
    static int kbhit(void);
};

#endif //_DXL_HPP_
