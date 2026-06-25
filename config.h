#pragma once

#include <libssh/libssh.h> //ssh库
#include <unistd.h>  //sleep()
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <fstream>
#include <cstdlib>
#include <pthread.h>
#include <signal.h>
#include <semaphore.h>

 // 要打印文件名，必须定义这个宏
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

// #include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/async.h"
#include <experimental/filesystem>

#define cmd_file_delete_set 1 //cmd_file_delete函数是否开启

#define MAX_underwater_acoustic_communication  10   //可接入水声通信设备最大值
#define MAX_decive_number 10   //可接入底层设备最大值
#define sleeptime_wait_communication 60 //等待通信模块上电时间 

#define MAX_CHILD_GET_PIC_WHILE_OVERTIME_SEC 10  //子进程采集图片循环超时时间 min*60s
#define MAX_FATHER_ACOUSYIC_WHILE_OVERTIME_SEC 20*60  //父进程水声通讯循环超时时间 min*60s
#define MAX_MAINTHREAD_ACOUSYIC_TASK_SLEEP_SEC 1*60  //主线程水声通任务讯循环时间 min*60s
#define MAX_STATE_CONVERT_OVERTIME_SEC 1  //数据接收状态转换超时时间 1s

#define acoustic_data_len 89  //水声通信数据长
#define timer_restart_cmd_len 5  //定时器复位命令数据长
#define max_repeat_count 3  //最大重发次数
#define acoustic_response_overtime 60  //水声通信等待回应超时时间 秒

#define sink_float_cmd_len 11 //沉浮通信数据长

#define cmd_file "cmd/cmd.txt"  //指令文件
#define cmd_config_file "cmd/cmdconfig.txt"  //指令另存为配置文件
#define cmd_file_save_path "cmd/save"  //指令另存为路径

#define cloud_config_file "sendtocloud/dataconfig.txt"  //云文件名配置文件
#define cloud_file_save_path "sendtocloud/save"  //云文件保存路径

#define sinkfloat_config_file "sinkfloat/sinkfloat_data_config.txt"  //沉浮文件名配置文件路径
#define sinkfloat_file_save_path "sinkfloat/save"  //沉浮文件保存路径

#define picture_config_file "picture/pictureconfig.txt" //图片保存的配置文件
#define picture_local_addr "picture/save"//图片保存路径初始化（缺少编号）

#define uart1_path "/dev/ttyS1" //串口设备路径(水声1)
#define uart2_path "/dev/myttyUSB1" //串口设备路径(水声2)
#define uart3_path "/dev/myttyUSB2" //串口设备路径(沉浮)
#define control_board_uart_path "/dev/myttyUSB0" //继电器mcu板串口设备路径
#define timer_uart_path "/dev/myttyUSB3" //定时器串口设备路径
/**************下载指令状态***********************/
/*cmd_state*/
#define cmd_no_get   0 //还没获取指令
#define cmd_none     1 //无指令传递
#define cmd_get      2 //获取到指令
#define cmd_finish   3 //指令传输完成

/**************水声通信数据功能位***********************/
/*数据帧 [2]位 */
#define acoustic_data_function_response_UD        0xFF //应答信号        --无有效数据      上下
#define acoustic_data_function_sample_D           0x00 //请求数据采集指令  --无有效数据      下
#define acoustic_data_function_cmdtransmit_D      0x01 //控制指令下传     --需携带有效数据   下
#define acoustic_data_function_timer1_D           0x02 //定时器复位1      --无有效数据      下
#define acoustic_data_function_timer2_D           0x03 //定时器复位2      --需携带有效数据   下
#define acoustic_data_function_datatransmit_U     0x04 //数据上传         --需携带有效数据  上
#define acoustic_data_function_dataretransmit_D   0x05 //重传当前包       --无有效数据      下
/**************水声通信接收数据状态***********************/
/*acoustic_receiverdata_state*/
#define acoustic_receivedata_state_waitdata        0x00 //等待回传数据帧
#define acoustic_receivedata_state_waitresponse    0x01 //等待回应帧

/**************水声通信 数据帧***********************/
#define acoustic_protocol_frame_block_num_index          4  //块编号下标
#define acoustic_protocol_frame_block_total_index        5  //块总数下标
#define acoustic_protocol_frame_vaild_length_index       6  //有效数据长度下标
#define acoustic_protocol_frame_first_vaild_data_index   7  //有效数据首位下标
/**************串口接收 状态机***********************/
enum  UART_STATUS{
    data_none,//没有数据
    data_received_one,//接收到了5A
    data_receiving,//接收剩下的数据
    data_receive_done//数据接收完成
    };
/*************************************************/
using namespace std;

/*
username        设备用户名 可以是root
remote_ip       设备ip
cmd_addr        设备上保存指令的地方
picture_addr    设备上保存图片的位置
data_addr       设备上保存数据的位置
// local_addr      本地保存位置（.为当前程序运行位置）
picture_number  采集图片张数
*/
typedef struct MyDevice {
    string username;     //设备用户名 可以是root
    string remote_ip;    //设备ip
    string cmd_addr;    //设备上保存指令的地方
    vector<string> picture_addr; //设备上保存图片的位置
    string data_addr;    //设备上保存数据的位置
    string log_addr;     //设备上保存日志的位置
    // string local_addr;   //本地保存位置（.为当前程序运行位置）
    int picture_number;   //采集图片张数
}mydevices;//接入设备


typedef struct MyAcousticDevice {
    int number;          //水声设备编号 从机编号 1 2 3 。。。
    string attribute;    //基站属性

}myacousticdevice;//接入水声设备

extern myacousticdevice acousticdevice[MAX_underwater_acoustic_communication];
extern mydevices device[MAX_decive_number]; //接入设备数组
extern mydevices cloudserver; //云服务器

extern int bottom_decive_number; //底层设备数
extern int acoustic_decive_number; //底层设备数
extern int cmd_state ; //指令状态
extern int cmd_dev_num;
extern string picture_save_path ;//图片保存路径

extern sem_t sem_satellite;//卫星使用信号量,父子进程间同步
extern sem_t sem_sinkfloat_thread_open; //沉浮控制的信号量
extern volatile int is_statelite_open;//卫星开启标志位
extern volatile int sinking_and_floating_module_mode ;//沉浮模块模式


extern uint8_t acoustic_receive_string[acoustic_data_len] ;      //水声通信接收数组
extern uint8_t acoustic_receive_string_vaild[acoustic_data_len]; //水声通信接收数组有效数据
extern volatile uint8_t acoustic_receive_string_valid_length ;   //水声通信接收数组有效数据长度

extern std::shared_ptr<spdlog::logger> console;//终端
extern std::shared_ptr<spdlog::logger> mylogger ;//日志文件
extern string log_path;//logs//logx.log;日志地址


