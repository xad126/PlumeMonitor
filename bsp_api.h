#pragma once
#include "fileopration.h"
#include "config.h"
#include <signal.h>

#include <iostream>
#include <string>



using namespace std;

typedef struct Acoustic_Task{ 
    // int cmd_num;               //需要下传的指令数
    // bool flag_cmd_finished;    //指令下传完成
    bool flag_data_finished;   //数据采集完成
    bool flag_timer_finished;  //定时器复位完成
}myacoustic_task;

typedef struct Cmd_Task{
    int devnum;               //需要下传的设备编号
    int cmd_num;                  //需要下传的指令数
    bool flag_cmd_finished;       //指令下传完成
    vector<string> cmd_vec;
    Cmd_Task():cmd_num(0),flag_cmd_finished(false),cmd_vec(0){}
}mycmd_task;

/*
pic_num        采集图片的数量
dev_ip         设备ip
dev_username   设备用户名
pic_addr       指令下传完成
*/
typedef struct Bottom_Base_Station_Task{ 
    int pic_num;               //采集图片的数量
    string dev_ip;             //设备ip
    string dev_username;       //设备用户名
    vector<string> pic_addr;   //指令下传完成
    Bottom_Base_Station_Task():pic_num(0),dev_ip(""),dev_username(""),pic_addr(0){}
}mybottom_base_station_task;

void check_cloud_dataconfig(void);//检查当前dataconfig的名字编号
void check_sinkfloat_dataconfig(void);//检查当前dataconfig的名字编号

string get_timer_val(void);
void timer_rectification(string timer_val);
void signal_handler(int sig);//中断信号

void open_peripheral(void);
void close_peripheral(void);

void open_sinkfloat(void);
void close_sinkfloat(void);

void wakeup_under_dev(void);
void sleep_under_dev(void);
bool check_ssh_connection(string ip, string username);

void* main_fun_thread(void* arg);
void* side_fun_thread(void* arg);
void* sinkfloat_fun_thread(void* arg);

extern myacoustic_task acoustic_task[MAX_underwater_acoustic_communication];
// extern mycmd_task cmd_task[MAX_underwater_acoustic_communication];
extern vector<mycmd_task*> cmd_task_vector;
