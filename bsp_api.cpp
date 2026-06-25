#include "fileopration.h"
#include "config.h"
#include "bsp_api.h"
#include "uart.h"
#include "SysFun_Crc.h"

#include <condition_variable> // std::condition_variable, std::cv_status
#include <termios.h>
#include <dirent.h>
#include <cmath>
#include "socket.h"
#include <png.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_RESIZE_IMPLEMENTATION
#include "stb_image_resize2.h"

// #include "logger.h"
bool flag_read_port_overtime = false; //读串口超时标志位
volatile bool flag_read_thread_exit = false;  //读线程退出标志位

/*信号量*/
sem_t sem_satellite;//卫星使用信号量,线程间同步
sem_t sem_sinkfloat_thread_open; //沉浮控制的信号量
/*****************************/
/*****  条件变量必要的参数  *****/
/*****************************/
// //父进程水声通信流程互斥锁
// pthread_mutex_t fatherpro_acoustic_mutex;
// //父进程水声通信流程条件变量
// pthread_cond_t fatherpro_acoustic_cond;


//主进程进程水声通信流程互斥锁
std::mutex mainthread_acoustic_mutex;
//创建一个条件变量
condition_variable mainthread_acoustic_cond;
/*****************************/

/*********************/
/*****  串口参数  *****/
/*********************/
int fd_uart1;
int fd_uart2;
int fd_uart3;
int fd_uart4;

uint8_t read_string[200] = "";
uint8_t acoustic_send_string[acoustic_data_len] = "";  //水声通信发射数组
uint8_t acoustic_receive_string[acoustic_data_len] = "";  //水声通信接收数组
uint8_t acoustic_receive_string_vaild[acoustic_data_len] = "";  //水声通信接收数组有效数据
uint8_t cmd_string_vaild[acoustic_data_len] = "";  //定时器命令有效数据
uint8_t timer_restart_cmd[timer_restart_cmd_len] = {0xFF,0x00,0x40,0x0D,0x0A};  //定时器复位指令
volatile uint8_t acoustic_receive_string_valid_length = 0;
volatile uint8_t cmd_string_valid_length = 0;
ssize_t res_length;

/*接收数据判断标志     <当前传输的从机标号,数据状态>*/
pair<int,int> acoustic_receiverdata_state = {0,acoustic_receivedata_state_waitdata};

FILE* fd_cloud;
FILE* fd_sinkfloat;
// DIR *dir_cloud;
int cloud_file_number;
int sinkfloat_file_number;
string sinkfloat_file_name ;//datax.txt
string cloud_file_name ;//datax.txt
string cloud_file_path ;//sendtocloud/save/datax.txt
string sinkfloat_file_path ;//sinkfloat/save/datax.txt
string log_path;//logs//logx.log;
string sink_float_depth={};//沉浮深度
float sink_float_depth_f;
string sink_float_flow={};//沉浮流量
/*********************/

/**
* @brief 开外设 (光纤，交换机)
* @return void
**/
void open_peripheral(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:peripheral open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:peripheral open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:peripheral open %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:peripheral open [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:peripheral open [{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t open_peripheral_cmd[5]={0xFF,0x00,0x03,0x42,0x40};
    write_port(fd_uart, open_peripheral_cmd, sizeof(open_peripheral_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}

/**
* @brief 关外设 (光纤，交换机)
* @return void
**/
void close_peripheral(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:peripheral close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:peripheral close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:peripheral close %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:peripheral close [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:peripheral close [{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t close_peripheral_cmd[5]={0xFF,0x00,0x04,0x43,0x40};
    write_port(fd_uart, close_peripheral_cmd, sizeof(close_peripheral_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}

/**
* @brief 开沉浮模块 
* @return void
**/
void open_sinkfloat(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:sinkfloat open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:sinkfloat open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:peripheral open %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:sinkfloat open [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:sinkfloat open [{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t open_sinkfloat_cmd[5]={0xFF,0x00,0x05,0x44,0x40};
    write_port(fd_uart, open_sinkfloat_cmd, sizeof(open_sinkfloat_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}

/**
* @brief 关沉浮模块
* @return void
**/
void close_sinkfloat(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:sinkfloat close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:sinkfloat close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:peripheral close %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:sinkfloat close [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:sinkfloat close [{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t close_sinkfloat_cmd[5]={0xFF,0x00,0x06,0x45,0x40};
    write_port(fd_uart, close_sinkfloat_cmd, sizeof(close_sinkfloat_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}

/**
* @brief 唤醒中继站 (继电器拉高)
* @return void
**/
void wakeup_under_dev(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:under_dev wakeup {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:under_dev wakeup {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:under_dev wakeup %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:under_dev wakeup [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:under_dev wakeup [{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t wakeup_under_dev_cmd[5]={0xFF,0x00,0x07,0x46,0x40};
    write_port(fd_uart, wakeup_under_dev_cmd, sizeof(wakeup_under_dev_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}

/**
* @brief 休眠中继站 (继电器拉低)
* @return void
**/
void sleep_under_dev(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:under_dev sleep {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:under_dev sleep {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:under_dev sleep%s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:under_dev sleep [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:under_dev sleep[{0}] failed.", control_board_uart_path);
        return;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t sleep_under_dev_cmd[5]={0xFF,0x00,0x08,0x47,0x40};
    write_port(fd_uart, sleep_under_dev_cmd, sizeof(sleep_under_dev_cmd));
    usleep(1000*500);
    close_port(fd_uart);
}
/**
* @brief 开启卫星
* @return 1
**/
int open_statelite(){
    int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:statelite open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:statelite open {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:statelite open %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:statelite open [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:statelite open [{0}] failed.", control_board_uart_path);
        return 0;
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t open_statelite_cmd[5]={0xFF,0x00,0x01,0x40,0x40};
    write_port(fd_uart, open_statelite_cmd, sizeof(open_statelite_cmd));
    usleep(1000*500);
    close_port(fd_uart);
    return 1;
}

/**
* @brief 关闭卫星
* @return 1
**/
int close_statelite(){
     int fd_uart;
    /***************** 串口操作 *****************/
    if( (fd_uart = open_port(control_board_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "main:statelite close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        SPDLOG_LOGGER_INFO(console, "main:statelite close {0} success, fd = {1:d}.", control_board_uart_path, fd_uart);
        //printf("main:statelite close %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "main:statelite close [{0}] failed.", control_board_uart_path);
        SPDLOG_LOGGER_ERROR(console, "main:statelite close [{0}] failed.", control_board_uart_path);
        return 0; 
    }
    /* config port */
    config_port(fd_uart, B115200);
    /*******************************************/
    uint8_t close_statelite_cmd[5]={0xFF,0x00,0x02,0x41,0x40};
    write_port(fd_uart, close_statelite_cmd, sizeof(close_statelite_cmd));
    usleep(1000*500);
    close_port(fd_uart);
    return 1;
}

bool check_ssh_connection(string ip, string username){
    ssh_session my_ssh_session = ssh_new();

    if (my_ssh_session == nullptr) {
        std::cerr << "Failed to create SSH session." << std::endl;
        return 1;
    }
    ssh_options_set(my_ssh_session, SSH_OPTIONS_HOST, ip.c_str());
    ssh_options_set(my_ssh_session, SSH_OPTIONS_USER, username.c_str());
    // Optional: set port if it's not the default 22
    // ssh_options_set(my_ssh_session, SSH_OPTIONS_PORT, "your_port_number");
 
    int rc = ssh_connect(my_ssh_session);
    if (rc != SSH_OK) {
        std::cerr << "SSH connection failed: " << ssh_get_error(my_ssh_session) << std::endl;
        ssh_free(my_ssh_session);
        return false;
    }

    std::cout << ip <<" SSH connection succeeded." << std::endl;
    ssh_disconnect(my_ssh_session);
    ssh_free(my_ssh_session);
    return true;
}
/*******************************************/
/********** 检查当前dataconfig的名字编号*******/
/*******************************************/
void check_cloud_dataconfig(){
    char sin_char[100];
    
    /*检测文件名*/
    if((fd_cloud = fopen(cloud_config_file,"r+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening dataconfig.txt");
        //cerr<<"Error opening dataconfig.txt"<<endl;
    }
    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, fd_cloud) != nullptr) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
        string sin = sin_char;
        if (sin_char[0] == '/' && sin_char[1] == '/') {
            continue; //  开头是//就跳过
        } 
        if (sin.find(':') == -1) {
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "number") {
            cloud_file_number = stoi(b)+1;
            continue;
        }
    }
    cloud_file_name = "data" + to_string(cloud_file_number) + ".txt";
    string temp = "/" + cloud_file_name;
    cloud_file_path = cloud_file_save_path + temp;
    string write_temp = "number:" + to_string(cloud_file_number);
    fseek(fd_cloud,0,SEEK_SET);
    fwrite(write_temp.c_str(),1,write_temp.size(),fd_cloud);
    fclose(fd_cloud);
}

/*******************************************/
/********** 检查当前sinkfloat_data_config的名字编号*******/
/*******************************************/
void check_sinkfloat_dataconfig(){
    char sin_char[100];
    
    /*检测文件名*/
    if((fd_sinkfloat = fopen(sinkfloat_config_file,"r+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening sinkfloat_dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening sinkfloat_dataconfig.txt");
        //cerr<<"Error opening dataconfig.txt"<<endl;
    }
    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, fd_sinkfloat) != nullptr) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
        string sin = sin_char;
        if (sin_char[0] == '/' && sin_char[1] == '/') {
            continue; //  开头是//就跳过
        } 
        if (sin.find(':') == -1) {
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "number") {
            sinkfloat_file_number = stoi(b)+1;
            continue;
        }
    }
    sinkfloat_file_name = "data" + to_string(sinkfloat_file_number) + ".txt";
    string temp = "/" + sinkfloat_file_name;
    sinkfloat_file_path = sinkfloat_file_save_path + temp;
    string write_temp = "number:" + to_string(sinkfloat_file_number);
    fseek(fd_sinkfloat,0,SEEK_SET);
    fwrite(write_temp.c_str(),1,write_temp.size(),fd_sinkfloat);
    fclose(fd_sinkfloat);
}

/**
* @brief 检查本地是否存在cmd.txt
* @return bool
**/
bool check_cmd_file(string file){
    ifstream fp(file);
    if(fp.is_open()){
        cmd_state =  cmd_get;
        fp.close();
        return true;
    }
    else{
        cmd_state =  cmd_none;
        fp.close();
        return false;
    }

}

/**
* @brief 解析指令
* @return 返回下传指令的设备数
**/
int analysis_cmd_file(string file){
    FILE* cmd_fp = nullptr;
    char sin_char[200] = {0}; //读行缓存
    int dev_num = 0;

    if((cmd_fp = fopen(file.c_str(), "r")) == nullptr){
        SPDLOG_LOGGER_ERROR(mylogger, "{} no find!",file);
        SPDLOG_LOGGER_ERROR(console, "{} no find!",file);
        //cerr << file <<" no find!" << endl;
   
        return dev_num;
    }

    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, cmd_fp) != nullptr) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
            
        string sin = sin_char;
        if (sin_char[0] == '/' && sin_char[1] == '/') {
            continue; //  开头是//就跳过
        } 
        if (sin.find(':') == -1) {
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "cmd_number") {
            dev_num = stoi(b);
            continue;
        }
        else if (a == "sinking and floating module mode"){
            sinking_and_floating_module_mode = stoi(b);
            sem_post(&sem_sinkfloat_thread_open);
            continue;
        }
        else if (a == "depth"){
            sink_float_depth= b;
            sink_float_depth_f=stof(b);
            continue;
        }
        else if (a == "flow"){
            sink_float_flow = b;
            continue;
        }
        else{
            mycmd_task *temp= new Cmd_Task();
            temp->devnum = stoi(a);//设备编号
            string temp1 = b.substr(0,b.find(' ')); 
            string temp2 = b.substr(b.find(' ')+1,b.size()- b.find(' ')-1);
            SPDLOG_LOGGER_INFO(mylogger, "timer change_cmd:{}",temp2);
            SPDLOG_LOGGER_INFO(console, "timer change_cmd:{}",temp2);
            temp->cmd_num = stoi(temp1);//指令数
            temp->flag_cmd_finished = false;//指令传输完成标志位
            for(int i = stoi(temp1); i > 0; i--){
                if(temp2.empty()) break;
                string cmd = temp2.substr(0,temp2.find('#'));
                temp2 = temp2.erase(0,temp2.find('#')+1);
                temp->cmd_vec.push_back(cmd);
            }
            cmd_task_vector.push_back(temp);
        }
    }
    fclose(cmd_fp);

    return dev_num;
}
/**
* @brief 指令文件转存
* @return void
**/
void cmd_file_save(string file,string filepath,string configfile){
    FILE* config_fp = nullptr;
    char sin_char[200] = {0}; //读行缓存
    int cmd_number = 0; //读取cmd数
    /*******************************读config.txt*******************************************/
    if((config_fp = fopen(configfile.c_str(), "r+")) == nullptr){
        SPDLOG_LOGGER_ERROR(mylogger, "{} no find!",configfile);
        SPDLOG_LOGGER_ERROR(console, "{} no find!",configfile);
        //cerr << configfile <<" no find!" << endl;
        return;
    }

    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, config_fp) != nullptr) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
        string sin = sin_char;
        if (sin_char[0] == '/' && sin_char[1] == '/') {
            continue; //  开头是//就跳过
        } 
        if (sin.find(':') == -1) {
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "number") {
            cmd_number = stoi(b);
            continue;
        }
    }
    const char* w  = to_string(cmd_number+1).c_str();
    fseek(config_fp, 0, SEEK_SET);
    fprintf(config_fp, "number:%s",w);
    fclose(config_fp);
    /*******************************复制文件*******************************************/
    string source = file;
    string destination = filepath + "/cmd" + to_string(cmd_number+1) + ".txt";

    ifstream src(source, ios::binary);       //ifstream不会创建文件

    ofstream dest(destination, ios::binary); //ofstream会创建文件

    dest << src.rdbuf();

    SPDLOG_LOGGER_INFO(mylogger, "cmd_file_save success");
    SPDLOG_LOGGER_INFO(console, "cmd_file_save success");
}

/**
* @brief 指令文件删除
* @return void
**/
void cmd_file_delete(string file){
    remove(file.c_str());
}
/**
* @brief 中断服务函数signal_handler
* @return void
**/
void signal_handler(int sig)
{
    /*解析文件命令*/
    if(sig == SIGUSR1){
        /*检查是否存在指令*/
        if(!check_cmd_file(cmd_file)){
            /*指令文件不存在*/
            cmd_state = cmd_none;
            SPDLOG_LOGGER_ERROR(mylogger, "{0} not find.[cmd_state:{1}]",cmd_file,cmd_state);
            SPDLOG_LOGGER_ERROR(console, "{0} not find.[cmd_state:{1}]",cmd_file,cmd_state);
            //cerr<< cmd_file << " not find"<< endl;
            return;
        } 
        SPDLOG_LOGGER_INFO(mylogger, "local {0} exsit.[cmd_state:{1}]",cmd_file,cmd_state);
        SPDLOG_LOGGER_INFO(console, "local {0} exsit.[cmd_state:{1}]",cmd_file,cmd_state);
        /*解析指令*/
        cmd_dev_num = analysis_cmd_file(cmd_file);
        if(cmd_dev_num){
            cmd_state =  cmd_get;  //接受到指令并且不为空
        }
        else{
            cmd_state =  cmd_none;  //指令为空
        }
    
        /*指令文件转存*/
        cmd_file_save(cmd_file,cmd_file_save_path,cmd_config_file);

        /*指令原文件删除*/
        #if cmd_file_delete_set
        cmd_file_delete(cmd_file);
        #endif
        cmd_state = cmd_get;
    }
}

bool check_cmd_finished(){
    for(auto cmd_dev : cmd_task_vector){
        if(!cmd_dev->flag_cmd_finished){
            return false;
        }
    }
    return true;
}
void set_acoustic_send_data(uint8_t function,uint8_t slave_number,uint8_t block_cur_num,uint8_t block_total_num,string cmd,uint8_t* data){
    // uint8_t data[89];
    if(cmd.size() > acoustic_data_len - 9) {
        SPDLOG_LOGGER_ERROR(mylogger, "cmd length is too long");
        SPDLOG_LOGGER_ERROR(console, "cmd length is too long");
        //cerr << "cmd length is too long" << endl;
        return ;
    }
    /*清除 置0*/
    bzero(data,acoustic_data_len);
    // memset(data,0,acoustic_data_len);
    /*帧头*/
    data[0] = 0x5A;
    data[1] = 0xA5;
    /*功能位*/
    data[2] = function;
    /*设备编号*/
    data[3] = slave_number;
    /*块编号*/
    data[4] = block_cur_num;
    /*总块编号*/
    data[5] = block_total_num;
    /*有效帧长*/
    data[6] = cmd.size();
    // /*有效数据长*/
    // data[5] = cmd.size();

    if(function == acoustic_data_function_cmdtransmit_D || \
       function == acoustic_data_function_timer2_D || \
       function == acoustic_data_function_datatransmit_U){
        /*指令内容*/
        for(int i = 0; i < cmd.size(); i++){
            data[7+i] = cmd[i];
        }
    }
    /*帧尾*/
    data[acoustic_data_len-1] = 0x40;

    /*校验*/
    data[acoustic_data_len-2] = 0x40;
    for(int i = 0; i < acoustic_data_len-2; i++){
        data[acoustic_data_len-2]+= data[i];
    }
}

/**********************************************/
/********** 检查接收数据是否所需 response帧*********/
/**********************************************/
bool check_receive_data_response(uint8_t* receive,int dev_num){
    if(receive[3] != acoustic_receiverdata_state.first || receive[2] != acoustic_data_function_response_UD){
        return false;
    }
    return true;
}
/*******************************************/
/********** 检查接收数据是否所需 data帧*********/
/*******************************************/
bool check_receive_data_old(uint8_t* receive,int dev_num){
    if(receive[3] != acoustic_receiverdata_state.first && receive[2] != acoustic_data_function_datatransmit_U){
        return false;
    }
    
    /*写入文件*/
    //a+ 以附加方式打开可读写的文件。
    //若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾后，即文件原先的内容会被保留。 
    //（原来的EOF符不保留）
    if((fd_cloud = fopen(cloud_file_path.c_str(),"a+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening dataconfig.txt");
        //cerr<<"Error opening dataconfig.txt"<<endl;
        return false;
    }
    if(receive[5]<=acoustic_data_len-8){
        /*data*/
        fwrite("data\n",1,strlen("data\n"),fd_cloud);
        /*acoustic_device_number acoustic_device_attribute*/
        if(dev_num>=1){
            string temp = to_string(acousticdevice[dev_num-1].number)+ " " +acousticdevice[dev_num-1].attribute+"\n";
            fwrite(temp.c_str(),1,temp.size(),fd_cloud);
        }
        fwrite(&receive[6],1,receive[5],fd_cloud);
        fwrite("\n",1,1,fd_cloud);
        fflush(fd_cloud);
    }
    else{
        return false;
    }
    fflush(fd_cloud);
    
    fclose(fd_cloud);

    return true;
}
bool check_receive_data(uint8_t* receive,int dev_num){
    /*检查 设备编号 功能号*/
    if(receive[3] != acoustic_receiverdata_state.first || receive[2] != acoustic_data_function_datatransmit_U){
        return false;
    }
    
    memcpy(acoustic_receive_string,receive,acoustic_data_len);
    /*截取有效数据*/
    acoustic_receive_string_valid_length = receive[acoustic_protocol_frame_vaild_length_index];
    bzero(acoustic_receive_string_vaild,sizeof acoustic_receive_string_vaild);
    memcpy(acoustic_receive_string_vaild,receive+acoustic_protocol_frame_first_vaild_data_index,acoustic_receive_string_valid_length);
    
    return true;
}
void cut_valid_cmd(uint8_t* cmd){
    /*检查 设备编号 功能号*/
    if(cmd[3] != acoustic_receiverdata_state.first || cmd[2] != acoustic_data_function_cmdtransmit_D){
        return;
    }
    
    memcpy(acoustic_receive_string,cmd,acoustic_data_len);
    /*截取有效数据*/
    cmd_string_valid_length = cmd[acoustic_protocol_frame_vaild_length_index];
    bzero(cmd_string_vaild,sizeof cmd_string_vaild);
    memcpy(cmd_string_vaild,cmd+acoustic_protocol_frame_first_vaild_data_index,cmd_string_valid_length);
}
/*********************************************************************************/
/************************** 检查接云服务器的cmd.txt是否存在 **************************/
/*********************************************************************************/
bool check_cloud_cmd_file(string file_path,string file_name,int opt){//opt 0:linux 1:windows
    FILE* fp;
    char tmp[256];
    string checkcmdfile_cmd;
    switch(opt){
        case 0 : {
            //checkcmdfile_cmd :ssh MR@10.249.53.227 \"ls C:/Users/MR/Desktop/DOWN_DATA/cmd\"
            checkcmdfile_cmd = "ssh "+ cloudserver.username + "@" + cloudserver.remote_ip +\
                            " \"ls " + file_path+ "\"";
        }break;

        case 1 : {
            for(auto& c : file_path){
                if(c=='/'){
                    c = '\\';
                }
            }
            //checkcmdfile_cmd :ssh MR@10.249.53.227 \"dir /b /a-d C:/Users/MR/Desktop/DOWN_DATA/cmd\"
            checkcmdfile_cmd = "ssh "+ cloudserver.username + "@" + cloudserver.remote_ip +\
                            " \"dir /b /a-d " + file_path+"\"";
        }break;

        default:{
            checkcmdfile_cmd = "ssh "+ cloudserver.username + "@" + cloudserver.remote_ip +\
                            " \"ls " + file_path+ "\"";
        }break;
    }
    
    fp = popen(checkcmdfile_cmd.c_str(), "r");
    while (fgets(tmp, sizeof(tmp), fp) != NULL) {
        /*判断cmd.txt*/
        if (tmp[strlen(tmp) - 1] == '\n' && tmp[strlen(tmp) - 2] == '\r'){
            int temp_len = strlen(tmp);
            tmp[temp_len - 1] = '\0';
            tmp[temp_len - 2] = '\0';
        }
        else if(tmp[strlen(tmp) - 1] == '\n' ){
            tmp[strlen(tmp) - 1] = '\0';
        }
            
        if(tmp[strlen(tmp) - 1] == '\r'){
            tmp[strlen(tmp) - 1] = '\0';
        }
        string filename_temp = tmp;
        if (filename_temp == file_name) {
            pclose(fp); 
            return true;
        }   
    }
    pclose(fp);
    return false;
}
void delete_cloud_cmd_file(string filepath){
    FILE* fp;

    string cloudcmdfileremove_cmd = "ssh "+ cloudserver.username + "@" + cloudserver.remote_ip +\
                            " \"rm " + filepath + "\"";
    fp = popen(cloudcmdfileremove_cmd.c_str(), "r");
    pclose(fp);
}
void acoustic_cmd_task(unique_lock<mutex>& lk){
    struct timespec timeout_acoustic_cmd_task;//任务延时
    cv_status cond_res;//条件变量返回值
    /***************************定时器更改采样频率任务**********************/
        /*如果检测到 cmd_state状态为 取到命令*/
        if(cmd_state == cmd_get){
            for(auto cmd_dev : cmd_task_vector){
                /*如果此设备的指令下传没有完成*/
                if(cmd_dev->flag_cmd_finished == false){
                    /*论寻发送指令个数*/
                    for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
                        /* 设置 发送内容*/
                        set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
                        /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
                        acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
                        /*重发次数*/
                        for(int j = 1; j < max_repeat_count; j++){
                            /*定时器复位*/
                            write_port(fd_uart4, timer_restart_cmd, timer_restart_cmd_len);
                            sleep(2);
                            /*更改定时器采样频率*/
                            cut_valid_cmd(acoustic_send_string);
                            cmd_string_vaild[cmd_string_valid_length]=0x0D;
                            cmd_string_vaild[cmd_string_valid_length+1]=0x0A;
                            write_port(fd_uart4, cmd_string_vaild, cmd_string_valid_length);
                            sleep(2);
                            SPDLOG_LOGGER_INFO(mylogger, "[thread main statelite]:timer changes frequency success {0:d}",cmd_string_valid_length);
                            SPDLOG_LOGGER_INFO(console, "[thread main statelite]:timer changes frequency success{0:d}",cmd_string_valid_length);
                        }
                        /*清除下发指令*/
                        (cmd_dev->cmd_vec).erase(cmd_it);
                    }
                }
            }
        }
//     /***************************指令任务**********************/
//     /*如果检测到 cmd_state状态为 取到命令*/
//     if(cmd_state == cmd_get){
//         for(auto cmd_dev : cmd_task_vector){
//             /*如果此设备的指令下传没有完成*/
//             if(cmd_dev->flag_cmd_finished == false){
//                 /*论寻发送指令个数*/
//                 for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
//                     /* 设置 发送内容*/
//                     set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
//                     /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
//                     acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
//                     /*重发次数*/
//                     for(int j = 0; j < max_repeat_count; j++){
//                         /*指令 发送指令 5A A5 01 dev.number 59 data 40*/
//                         write_port(fd_uart1, acoustic_send_string, acoustic_data_len);

//                         //等待条件满足  //解锁
//                         cond_res = mainthread_acoustic_cond.wait_for(lk,chrono::seconds(acoustic_response_overtime));

//                         /*取到回应信号*/
//                         if(cond_res == cv_status::no_timeout){
//                             /*取到回应*/
//                             cout << "[thread main]指令任务(卫星开启后):read response\n" << endl;
//                             /*清除下发指令*/
//                             (cmd_dev->cmd_vec).erase(cmd_it);
//                             break;
//                         }
//                         else if(j==max_repeat_count-1){
//                             cmd_it++;///////////////////////////////////////达到最大重发次数后还没有回应才++
//                         }
//                     }
//                 }
//             }
//         }
//     }
    /*检查指令下载状态*/
    for(auto cmd_dev : cmd_task_vector){
        if((cmd_dev->cmd_vec).empty()){
            cmd_dev->flag_cmd_finished = true;
        }
    }
    if(cmd_state == 2 && check_cmd_finished()){
        cmd_state = 3;
    }
 }
void data_file_write_contents(){
    //a+ 以附加方式打开可读写的文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾后，即文件原先的内容会被保留。 
    if((fd_cloud = fopen(cloud_file_path.c_str(),"a+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening dataconfig.txt");
        //cerr<<"Error opening dataconfig.txt"<<endl;
    }
    string w_temp;
    //写入 cmd log
    w_temp = "cmd log\n";
    fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
    w_temp.clear();
    //写入 cmd_state
    w_temp = "cmd_state:"+ to_string(cmd_state) + "\n";
    fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
    w_temp.clear();
    //写入指令下传状态
    for(auto cmd_dev : cmd_task_vector){
        /*节点编号 节点属性*/
        w_temp.clear();
        w_temp = to_string(cmd_dev->devnum) +" "+ acousticdevice[cmd_dev->devnum-1].attribute+"\n";
        fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);

        /*receive: load:*/
        w_temp.clear();
        w_temp = "receive:" + to_string(cmd_dev->cmd_num) + " load:"+to_string(cmd_dev->cmd_num-(cmd_dev->cmd_vec).size())+"\n";
        fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);

        /*rest cmd:*/
        if((cmd_dev->cmd_vec).size()>0){
            w_temp.clear();
            w_temp = "rest cmd:" ;
            for(auto cmd_string : cmd_dev->cmd_vec){
                w_temp+=cmd_string+"\n";
            }
            fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
        }
    }
    fflush(fd_cloud);
    fclose(fd_cloud);
}
/**
* @brief 接收数据处理 保存数据进入map
* @param receive_string:       输入 接收数组
* @param receive_string_vaild: 输入 有效数组
* @param valid_length:         输入 有效数组长度
* @param data:                 输出 块保存map
* @param block_num:            输出 当前块编号
* @param block_total:          输出 块总数
* @return 保存成功 true
**/
bool _processing_data_packet(uint8_t* receive_string,uint8_t* receive_string_vaild,uint8_t valid_length,
                            map<int,string>& data,int& block_num,int& block_total)
{
    /**/
    block_num = receive_string[acoustic_protocol_frame_block_num_index];
    block_total = receive_string[acoustic_protocol_frame_block_total_index];
    if(data.count(block_num)){
        return false;
    }
    else if(!data.empty() && (*data.rbegin()).first + 1 != block_num){
        return false;
    }
    string temp_data;
    for(int i = 0;i < valid_length;i++){
        temp_data +=receive_string_vaild[i];
    }

    data.insert({block_num,temp_data});
    return true;
}
/**
* @brief 接收数据处理 保存数据进入map
* @param file:           输入文件名
* @param data:           输入有效数组map
* @param block_total:    数据块总数
* @return 保存成功 true
**/
bool _total_data_write_to_file(const string& file, map<int, string>& data,uint8_t block_total,int dev_num){
    FILE *fp = nullptr;
    
    /*检测数据是否完整*/
    int i = 1;
    for(auto it=data.begin();it!=data.end();it++){
        if( (*it).first != i++ ){
            return false;
        }
    }
    if(block_total != i-1){
        return false;
    }
    /*将数据写入文件*/
    if((fp = fopen(file.c_str(),"a+")) == nullptr){
        SPDLOG_LOGGER_ERROR(mylogger, "Cannot open file: {}",file);
        SPDLOG_LOGGER_ERROR(console, "Cannot open file: {}",file);
        //cerr<< "Cannot open file:" << file << endl;
        return false;   
    }
    /*data*/
    fwrite("data\n",1,strlen("data\n"),fp);
    /*acoustic_device_number acoustic_device_attribute*/
    if(dev_num>=1){
        string temp = to_string(acousticdevice[dev_num-1].number)+ " " +acousticdevice[dev_num-1].attribute+"\n";
        fwrite(temp.c_str(),1,temp.size(),fp);
    }
    for(auto it=data.begin();it!=data.end();it++){
        fwrite(((*it).second).c_str() , 1,((*it).second).size(),fp);
    }
    fwrite("\n",1,strlen("\n"),fp);
    fflush(fp);
    fclose(fp);
    data.clear();
    return true;

}
/*******************************************/
/********** 闹钟信号（定时器）处理函数 *********/
/******************************************/
// void sig_handler(int signal) {
 
//     if(signal == SIGALRM){
//         printf("sig_handler: %d SIGALRM\n", signal);
//         flag_read_port_overtime = true;

//         /*唤醒休眠线程*/
//         pthread_cond_broadcast(&fatherpro_acoustic_cond);
    
//         /*解锁*/
//         pthread_mutex_unlock(&fatherpro_acoustic_mutex);
        
//     }
    
// }
/*****************************************************/
/************************读线程1************************/
/*****************************************************/
void* read_fun_thread_1(void* arg){
    bool (*datafun_temp)(uint8_t* data,int num);
    int uart_status=data_none;
    struct timespec curtime_readthread,oldtime_readthread;//任务延时
    // signal(SIGALRM, sig_handler);
    while(1){
        clock_gettime(CLOCK_REALTIME, &curtime_readthread);
        if(curtime_readthread.tv_sec-oldtime_readthread.tv_sec > MAX_STATE_CONVERT_OVERTIME_SEC){
            uart_status=data_none;
        } 
        /******************** 数据接收 状态机 *******************/
        switch(uart_status){
            case data_none: {
                memset(read_string,0,sizeof read_string);
                res_length = 0;
                res_length = read_port(fd_uart1, read_string, sizeof read_string, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);//记得补充一下，每次进行状态转移都需要gettime
                    break;
                }
                if(res_length>=2 && read_string[0] == 0x5A && read_string[1] == 0xA5 ){
                    uart_status=data_receiving;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
                else if(res_length==1 && read_string[0] == 0x5A ){
                    uart_status=data_received_one;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            };break;

            case data_received_one:{
                int tem_length=0;
                tem_length=res_length;
                res_length += read_port(fd_uart1, read_string+tem_length, (sizeof read_string)-tem_length, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);//记得补充一下，每次进行状态转移都需要gettime
                    break;
                }
                if(res_length>=2 && read_string[0] == 0x5A && read_string[1] == 0xA5 ){
                    uart_status=data_receiving;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
                else if(res_length==2 && read_string[1] != 0xA5 ){
                    uart_status=data_none;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            }break;

            case data_receiving:{
                int tem_length=0;
                tem_length=res_length;
                res_length+= read_port(fd_uart1, read_string+tem_length, (sizeof read_string)-tem_length, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            };break;
            
            case data_receive_done:{  
                    /*如果取到应答  帧头帧尾*/
                    if(read_string[0] == 0x5A && read_string[1] == 0xA5 && read_string[acoustic_data_len-1] == 0x40){
                        /*校验位检测  校验位*/
                        uint8_t check = read_string[acoustic_data_len-1];
                        for(int i = 0; i < acoustic_data_len-2; i++){
                            check+=read_string[i];
                        }
                        if(check == read_string[acoustic_data_len-2]){
                            /*数据处理函数选择*/
                            switch(acoustic_receiverdata_state.second){
                                case acoustic_receivedata_state_waitdata:{
                                    datafun_temp = check_receive_data;
                                };
                                break;
                                case acoustic_receivedata_state_waitresponse:{
                                    datafun_temp = check_receive_data_response;
                                };
                                break;
                                default:{
                                    datafun_temp = check_receive_data_response;
                                }
                                break;
                            }
                            /*数据检测：功能 编号 截取有效数据*/
                            if((*datafun_temp)(read_string,acoustic_receiverdata_state.first) == true){

                                /*唤醒主线程的waitfor*/
                                mainthread_acoustic_cond.notify_all();
                                usleep(5); //延时，缓冲一下，让os调度
                            }
                        }
                    }
                    // else{
                    //     memset(read_string,0,sizeof read_string);
                    // }
                    SPDLOG_LOGGER_INFO(mylogger, "[thread 1.2.1]:{O:ld} bytes read : ", res_length);
                    SPDLOG_LOGGER_INFO(console, "[thread 1.2.1]:{O:ld} bytes read : ", res_length);
                    //printf("[thread 1.2.1:%ld bytes read : ", res_length);
                    for (ssize_t i = 0; i < res_length; i++)
                    {
                        printf("%x ", (uint8_t)(read_string[i]));
                    }
                    printf("\n");
                    uart_status=data_none;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
            }break;
            default:{
                memset(read_string,0,sizeof read_string);
                res_length = 0;
                uart_status=data_none;
                clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
            }break;
            
        }  
         /*****************************************************************/
         if(flag_read_port_overtime==true){
            break;
         }

    }
    return NULL; 

}
/*****************************************************/
/************************读线程2************************/
/*****************************************************/
void* read_fun_thread_2(void* arg){
    bool (*datafun_temp)(uint8_t* data,int num);
    int uart_status=data_none;
    struct timespec curtime_readthread,oldtime_readthread;//任务延时
    // signal(SIGALRM, sig_handler);
    while(1){
        clock_gettime(CLOCK_REALTIME, &curtime_readthread);
        if(curtime_readthread.tv_sec-oldtime_readthread.tv_sec > MAX_STATE_CONVERT_OVERTIME_SEC){
            uart_status=data_none;
        } 
        /******************** 数据接收 状态机 *******************/
        switch(uart_status){
            case data_none: {
                memset(read_string,0,sizeof read_string);
                res_length = 0;
                res_length = read_port(fd_uart2, read_string, sizeof read_string, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);//记得补充一下，每次进行状态转移都需要gettime
                    break;
                }
                if(res_length>=2 && read_string[0] == 0x5A && read_string[1] == 0xA5 ){
                    uart_status=data_receiving;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
                else if(res_length==1 && read_string[0] == 0x5A ){
                    uart_status=data_received_one;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            };break;

            case data_received_one:{
                int tem_length=0;
                tem_length=res_length;
                res_length += read_port(fd_uart2, read_string+tem_length, (sizeof read_string)-tem_length, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);//记得补充一下，每次进行状态转移都需要gettime
                    break;
                }
                if(res_length>=2 && read_string[0] == 0x5A && read_string[1] == 0xA5 ){
                    uart_status=data_receiving;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
                else if(res_length==2 && read_string[1] != 0xA5 ){
                    uart_status=data_none;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            }break;

            case data_receiving:{
                int tem_length=0;
                tem_length=res_length;
                res_length+= read_port(fd_uart2, read_string+tem_length, (sizeof read_string)-tem_length, 1, 0);
                if(res_length>=89){
                    uart_status=data_receive_done;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
                    break;
                }
            };break;
            
            case data_receive_done:{  
                    /*如果取到应答  帧头帧尾*/
                    if(read_string[0] == 0x5A && read_string[1] == 0xA5 && read_string[acoustic_data_len-1] == 0x40){
                        /*校验位检测  校验位*/
                        uint8_t check = read_string[acoustic_data_len-1];
                        for(int i = 0; i < acoustic_data_len-2; i++){
                            check+=read_string[i];
                        }
                        if(check == read_string[acoustic_data_len-2]){
                            /*数据处理函数选择*/
                            switch(acoustic_receiverdata_state.second){
                                case acoustic_receivedata_state_waitdata:{
                                    datafun_temp = check_receive_data;
                                };
                                break;
                                case acoustic_receivedata_state_waitresponse:{
                                    datafun_temp = check_receive_data_response;
                                };
                                break;
                                default:{
                                    datafun_temp = check_receive_data_response;
                                }
                                break;
                            }
                            /*数据检测：功能 编号 截取有效数据*/
                            if((*datafun_temp)(read_string,acoustic_receiverdata_state.first) == true){

                                /*唤醒主线程的waitfor*/
                                mainthread_acoustic_cond.notify_all();
                                usleep(5); //延时，缓冲一下，让os调度
                            }
                        }
                    }
                    // else{
                    //     memset(read_string,0,sizeof read_string);
                    // }
                    SPDLOG_LOGGER_INFO(mylogger, "[thread 1.2.2]:{} bytes read : ", res_length);
                    SPDLOG_LOGGER_INFO(console, "[thread 1.2.2]:{} bytes read : ", res_length);
                    //printf("[thread 1.2.2]:%ld bytes read : ", res_length);
                    for (ssize_t i = 0; i < res_length; i++)
                    {
                        printf("%x ", (uint8_t)(read_string[i]));
                    }
                    printf("\n");
                    uart_status=data_none;
                    clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
            }break;
            default:{
                memset(read_string,0,sizeof read_string);
                res_length = 0;
                uart_status=data_none;
                clock_gettime(CLOCK_REALTIME, &oldtime_readthread);
            }break;
            
        }  
         /*****************************************************************/
          if(flag_read_port_overtime==true){
            break;
        }

    }
    return NULL; 

}
//vector<mycmd_task*> cmd_task_vector; //指令任务数组
/**
* @brief 主线程-轮询节点获取数据
    1.轮询节点
    2.定时器复位
    3.检测指令(下传指令)
    4.数据请求
    5.卫星传输
* @return void
**/
void* main_fun_thread(void* arg){
    FILE* popen_fp;
    int pclose_fp;
    pthread_t read_thread_1 ;  //读回应线程1
    pthread_t read_thread_2 ;  //读回应线程2
    struct timespec curtime_mainthread,oldtime_mainthread;//任务延时
    cv_status cond_res;//条件变量返回值

    // /***************** 串口操作 *****************/
    // if( (fd_uart1 = open_port(uart1_path)) > 0){
    //     SPDLOG_LOGGER_INFO(mylogger, "[thread main]:uart1 open {0} success, fd = {1:d}.", uart1_path, fd_uart1);
    //     SPDLOG_LOGGER_INFO(console, "[thread main]:uart1 open {0} success, fd = {1:d}.", uart1_path, fd_uart1);
    //     //printf("[thread main]:uart1 open %s success, fd = %d.\n", uart1_path, fd_uart1);
    // }
    // else{
    //     SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:uart1 open {0} failed.", uart1_path);
    //     SPDLOG_LOGGER_ERROR(console, "[thread main]:uart1 open {0} failed.", uart1_path);
    // }
    // /* config port */
    // config_port(fd_uart1, B115200);

    if( (fd_uart2 = open_port(uart2_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:uart2 open {0} success, fd = {1:d}.", uart2_path, fd_uart2);
        SPDLOG_LOGGER_INFO(console, "[thread main]:uart2 open {0} success, fd = {1:d}.", uart2_path, fd_uart2);
        //printf("[thread main]:uart2 open %s success, fd = %d.\n", uart2_path, fd_uart2);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:uart2 open {0} failed.", uart2_path);
        SPDLOG_LOGGER_ERROR(console, "[thread main]:uart2 open {0} failed.", uart2_path);
    }
    /* config port */
    config_port(fd_uart2, B9600);

    if( (fd_uart4 = open_port(timer_uart_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:uart4 open {0} success, fd = {1:d}.", timer_uart_path, fd_uart4);
        SPDLOG_LOGGER_INFO(console, "[thread main]:uart4 open {0} success, fd = {1:d}.", timer_uart_path, fd_uart4);
       // printf("[thread main]:uart4 open %s success, fd = %d.\n", timer_uart_path, fd_uart4);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:uart4 open {0} failed.", timer_uart_path);
        SPDLOG_LOGGER_ERROR(console, "[thread main]:uart4 open {0} failed.", timer_uart_path);
    }
    /* config port */
    config_port(fd_uart4, B9600);

    /***************************创建读线程********************************/
    //pthread_create(&read_thread_1, NULL, read_fun_thread_1,(void*)"读线程1");
    pthread_create(&read_thread_2, NULL, read_fun_thread_2,(void*)"读线程2");
    /**************独占锁上锁**************/
    unique_lock<mutex> lk(mainthread_acoustic_mutex);
    
    /*********************水声设备 任务*********************/
    vector<myacoustic_task> task_vector;  //task_vector[0] 对应编号为1 的从机（水声通信机）
    vector<myacousticdevice> acoustic_vector;//任务执行完会删除
    //任务数组初始化
    //水声设备初始化
    for(int i=0; i< acoustic_decive_number; i++){
        acoustic_vector.push_back(acousticdevice[i]);
        task_vector.push_back(acoustic_task[i]);
        task_vector[i].flag_data_finished = false;
        task_vector[i].flag_timer_finished = false;
    }
    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:task start.[while overtime:{0}s][acoustic response overtime:{1}s]", MAX_FATHER_ACOUSYIC_WHILE_OVERTIME_SEC,acoustic_response_overtime);
    SPDLOG_LOGGER_INFO(console, "[thread main]:task start.[while overtime:{0}s][acoustic response overtime:{1}s]", MAX_FATHER_ACOUSYIC_WHILE_OVERTIME_SEC,acoustic_response_overtime);
    /**********************任务开始***************************/
    clock_gettime(CLOCK_REALTIME, &oldtime_mainthread);
    while(1){
        
        /*如果数组为空 表示 数据请求-定时器复位 任务全完成*/
        if(acoustic_vector.empty()) break;
        /*超时退出*/
        clock_gettime(CLOCK_REALTIME, &curtime_mainthread);
        if(curtime_mainthread.tv_sec-oldtime_mainthread.tv_sec > MAX_FATHER_ACOUSYIC_WHILE_OVERTIME_SEC){
            break;  
        } 

        for(auto dev : acoustic_vector){
            // task_vector[0].flag_data_finished=true;
            // task_vector[0].flag_timer_finished=true;
            /****************************************数据采集未完成**********************************/
            if(task_vector[dev.number-1].flag_data_finished == false){
                /**数据采集开始 初始化**/
                map<int,string> temp_data_map;
                bool task_get_acoustic_data_status =  true;//采集数据任务流程的状态指示(true:任务正常执行 false:上一流程超时，结束任务)
                int block_num = 0,block_total = 0;

                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:dev[{}] start to get data.", dev.number);
                SPDLOG_LOGGER_INFO(console, "[thread main]:dev[{}] start to get data.", dev.number);

                /***************************** 流程1：发送数据请求-固定重发次数 **********************************/
                /*数据请求： 超时后继续重发请求信号（acoustic_data_function_sample_D）*/
                if(task_get_acoustic_data_status) { //流程标志位为true 则可执行流程
                    for(int i = 0; i < max_repeat_count; i++){
                        /* 设置 发送内容*/
                        set_acoustic_send_data(acoustic_data_function_sample_D,dev.number,1,1,"",acoustic_send_string);
                        /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
                        acoustic_receiverdata_state = {dev.number,acoustic_receivedata_state_waitdata};

                        /*采集 发送指令 5A A5 00 dev.number 59 00... 40*///水声设备分支条件
                        // if(dev.number==1){                               
                        //     write_port(fd_uart1, acoustic_send_string, acoustic_data_len);
                        // }
                        // else{
                            write_port(fd_uart2, acoustic_send_string, acoustic_data_len);
                        // }
                        SPDLOG_LOGGER_INFO(mylogger,"[thread main]:dev[{0:d}]流程1:采集指令发送.",dev.number);
                        SPDLOG_LOGGER_INFO(console,"[thread main]:dev[{0:d}]流程1:采集指令发送.",dev.number);

                        //等待条件满足  //解锁
                        cond_res = mainthread_acoustic_cond.wait_for(lk,chrono::seconds(acoustic_response_overtime));
                        
                        /*取到回应信号*/
                        if(cond_res == cv_status::no_timeout){
                            /*数据处理 保存*/
                            if(_processing_data_packet(acoustic_receive_string,acoustic_receive_string_vaild,
                                acoustic_receive_string_valid_length,temp_data_map,block_num,block_total)){
                                /*取到回应*/
                                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:dev[{0:d}]流程1:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                SPDLOG_LOGGER_INFO(console, "[thread main]:dev[{0:d}]流程1:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                //cout << "[thread main]流程1:readed data - 设备号："<<dev.number<<"数据:"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                // task_vector[dev.number-1].flag_data_finished = true;
                                task_get_acoustic_data_status = true;
                                break;
                            }
                        }
                        else{
                            SPDLOG_LOGGER_ERROR(mylogger,"[thread main]:dev[{0:d}]流程1:No readed data - 数据请求超时.",dev.number);
                            SPDLOG_LOGGER_ERROR(console, "[thread main]:dev[{0:d}]流程1:No readed data - 数据请求超时.",dev.number);
                            //cout << "[thread main]流程1:No readed data - 数据请求超时."<<  endl;
                            task_get_acoustic_data_status = false;
                            continue;
                        }
                        
                    }
                }
                /***************************** 流程2：回应应答信号 **********************************/
                /*回应数据请求： 超时后继续重发回应信号（acoustic_data_function_response_UD）*/
                if(task_get_acoustic_data_status) { //流程标志位为true 则可执行流程
                    
                    if(block_total == 1 && !temp_data_map.empty()){
                    /*只有一包数据*/
                        task_vector[dev.number-1].flag_data_finished = true;
                        /*数据写入*/
                        _total_data_write_to_file(cloud_file_path,temp_data_map,block_total,dev.number);
                        SPDLOG_LOGGER_INFO(mylogger,"[thread main]:dev[{0:d}]流程2:采集任务完成 数据写入.",dev.number);
                        SPDLOG_LOGGER_INFO(console,"[thread main]::dev[{0:d}]流程2:采集任务完成 数据写入.",dev.number);
                        //cout << "[thread main]流程2:采集任务完成 数据写入."<<  endl;
                    }
                    else{
                    /*多包数据*/    
                        int _temp_block_cnt = 1;
                        while(_temp_block_cnt <= block_total){
                            /*退出条件1 采集完数据*/
                            if(task_vector[dev.number-1].flag_data_finished == true){
                                /*数据写入*/
                                _total_data_write_to_file(cloud_file_path,temp_data_map,block_total,dev.number);
                                SPDLOG_LOGGER_INFO(mylogger,"[thread main]:dev[{0:d}]流程2:采集任务完成 数据写入.",dev.number);
                                SPDLOG_LOGGER_INFO(console,"[thread main]:dev[{0:d}]流程2:采集任务完成 数据写入.",dev.number);        
                                //cout << "[thread main]流程2:采集任务完成 数据写入."<<  endl;
                                break;
                            }
                            /*退出条件2 上一部流程未完成 终止采集任务*/
                            if(!task_get_acoustic_data_status) {
                                SPDLOG_LOGGER_ERROR(mylogger,"[thread main]:dev[{0:d}]流程2:上一部流程未完成 终止采集任务.",dev.number);
                                SPDLOG_LOGGER_ERROR(console,"[thread main]:dev[{0:d}]流程2:上一部流程未完成 终止采集任务.",dev.number);
                                //cout << "[thread main]流程2:上一部流程未完成 终止采集任务."<<  endl;
                                break;
                            }

                            /*****************************发送回应帧并等待*****************************/
                            /* 设置 发送内容*/
                            set_acoustic_send_data(acoustic_data_function_response_UD,dev.number,1,1,"",acoustic_send_string);
                            /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
                            acoustic_receiverdata_state = {dev.number,acoustic_receivedata_state_waitdata};
                            /************************循环发送************************/
                            for(int i = 0; i < max_repeat_count; i++){
                            
                                /*采集 发送指令 5A A5 00 dev.number 59 00... 40*///水声设备分支条件
                                // if(dev.number==1){                               
                                // write_port(fd_uart1, acoustic_send_string, acoustic_data_len);
                                // }
                                // else{
                                write_port(fd_uart2, acoustic_send_string, acoustic_data_len);
                                // }
                                SPDLOG_LOGGER_INFO(mylogger,"[thread main]:dev[{0:d}]流程2:采集指令发送.",dev.number);
                                SPDLOG_LOGGER_INFO(console,"[thread main]:dev[{0:d}]流程2:采集指令发送.",dev.number);

                                //等待条件满足  //解锁
                                cond_res = mainthread_acoustic_cond.wait_for(lk,chrono::seconds(acoustic_response_overtime));
                                
                                /*取到回应信号*/
                                if(cond_res == cv_status::no_timeout){
                                    /*块号校验*/
                                    _temp_block_cnt++;
                                    
                                    if(_temp_block_cnt == acoustic_receive_string[4] && _temp_block_cnt !=block_total){
                                    /*非最后一包数据*/

                                        /*数据处理 保存*/
                                        if(_processing_data_packet(acoustic_receive_string,acoustic_receive_string_vaild,
                                        acoustic_receive_string_valid_length,temp_data_map,block_num,block_total)){
                                            /*取到回应*/
                                            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:dev[{0:d}]流程2:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            SPDLOG_LOGGER_INFO(console, "[thread main]:dev[{0:d}]流程2:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            //cout << "[thread main]流程2:readed data - 设备号："<<dev.number<<"数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                            /*跳出循环发送*/
                                            break;
                                        }
                                        else{
                                            SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:dev[{0:d}]流程2:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            SPDLOG_LOGGER_ERROR(console, "[thread main]:dev[{0:d}]流程2:readed data - 数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            //cout << "[thread main]流程2:readed data [error]- 设备号："<<dev.number<<"数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                            task_get_acoustic_data_status = false;
                                        }
                                    }
                                    else if(_temp_block_cnt == acoustic_receive_string[4] && _temp_block_cnt ==block_total){
                                    /*最后一包数据*/
                                        /*数据处理 保存*/
                                        if(_processing_data_packet(acoustic_receive_string,acoustic_receive_string_vaild,
                                        acoustic_receive_string_valid_length,temp_data_map,block_num,block_total)){
                                            /*取到回应*/
                                            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:dev[{0:d}]流程2:readed data - 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            SPDLOG_LOGGER_INFO(console, "[thread main]:dev[{0:d}]流程2:readed data - 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            //cout << "[thread main]流程2:readed data - 设备号："<<dev.number<<"最后数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                            task_vector[dev.number-1].flag_data_finished = true;
                                            /*跳出循环发送*/
                                            break;
                                        }
                                        else{
                                            SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:dev[{0:d}]流程2:readed data - 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            SPDLOG_LOGGER_ERROR(console, "[thread main]:dev[{0:d}]流程2:readed data - 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                            //cout << "[thread main]流程2:readed data [error]-设备号："<<dev.number<<" 最后数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                            task_get_acoustic_data_status = false;
                                        }
                                    }
                                    else{
                                    /*接收块号不正确*/
                                        SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:dev[{0:d}]流程2: 块号错误 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                        SPDLOG_LOGGER_ERROR(console, "[thread main]:dev[{0:d}]流程2: 块号错误 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                        //cout << "[thread main]流程2:readed data [error]- 块号错误 设备号："<<dev.number<<"数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                        task_get_acoustic_data_status = false;
                                    }
                                    
                                }
                                else{
                                    SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:dev[{0:d}]流程2:No readed response - 数据请求超时 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                    SPDLOG_LOGGER_ERROR(console, "[thread main]:dev[{0:d}]流程2:No readed response - 数据请求超时 最后数据:{1:d} 总:{2:d}",dev.number,block_num,block_total);
                                    //cout << "[thread main]流程2:No readed response - 数据请求超时 设备号："<<dev.number<<"数据"<<block_num<<" 总:"<<block_total<<"."<<  endl;
                                    task_get_acoustic_data_status = false;
                                }
                                
                            }
                        }
                    }
                    
                }
            }
            task_vector[dev.number-1].flag_timer_finished =true;
            
            // /***************************定时器复位未完成**********************/
            // if(task_vector[dev.number-1].flag_timer_finished == false){
            //     /* 设置 发送内容*/
            //     set_acoustic_send_data(acoustic_data_function_timer1_D,dev.number,1,1,"",acoustic_send_string);
            //     /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
            //     acoustic_receiverdata_state = {dev.number,acoustic_receivedata_state_waitresponse};
            //     for(int i = 0; i < max_repeat_count; i++){
            //         /*定时器复位 发送指令 5A A5 03 dev.number 59 data 40*///水声设备分支条件
            //         if(dev.number==1){                               
            //             write_port(fd_uart1, acoustic_send_string, acoustic_data_len);
            //             }
            //             else{
            //             write_port(fd_uart2, acoustic_send_string, acoustic_data_len);
            //             }
                    

            //         //等待条件满足  //解锁
            //         cond_res = mainthread_acoustic_cond.wait_for(lk,chrono::seconds(acoustic_response_overtime));
                    
            //         /*取到回应信号*/
            //         if(cond_res == cv_status::no_timeout){

            //             /*取到回应*/
            //             cout << "[thread main]设备号："<<dev.number<<"定时器复位:readed response."<<  endl;
            //             task_vector[dev.number-1].flag_timer_finished = true;
            //             break;

            //         }
            //         else{
            //             cout << "[thread main]设备号："<<dev.number<<"定时器复位:No readed response."<<  endl;

            //             continue;
            //         }
            //     }
            // }
        }
        

        /*检查acoustic_vector中元素是否删除*/
        for(auto iter=acoustic_vector.begin();iter!=acoustic_vector.end();){
            if(task_vector[(*iter).number-1].flag_data_finished == true && task_vector[(*iter).number-1].flag_timer_finished == true){
                acoustic_vector.erase(iter);
            }
            else
            {
                iter++;
            }
        }
        // /***************************定时器更改采样频率任务**********************/
        // /*如果检测到 cmd_state状态为 取到命令*/
        // if(cmd_state == cmd_get){
        //     for(auto cmd_dev : cmd_task_vector){
        //         /*如果此设备的指令下传没有完成*/
        //         if(cmd_dev->flag_cmd_finished == false){
        //             /*论寻发送指令个数*/
        //             for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
        //                 /* 设置 发送内容*/
        //                 set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
        //                 /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
        //                 acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
        //                 /*重发次数*/
        //                 for(int j = 1; j < max_repeat_count; j++){
        //                     /*定时器复位*/
        //                     write_port(fd_uart4, timer_restart_cmd, timer_restart_cmd_len);
        //                     sleep(2);
        //                     /*更改定时器采样频率*/
        //                     cut_valid_cmd(acoustic_send_string);
        //                     cmd_string_vaild[cmd_string_valid_length]=0x0D;
        //                     cmd_string_vaild[cmd_string_valid_length+1]=0x0A;
        //                     write_port(fd_uart4, cmd_string_vaild, cmd_string_valid_length+2);
        //                     sleep(2);
        //                     SPDLOG_LOGGER_INFO(mylogger, "[thread main while]:timer changes frequency success {0:d}",cmd_string_valid_length);
        //                     SPDLOG_LOGGER_INFO(console, "[thread main while]:timer changes frequency success {0:d}",cmd_string_valid_length);
        //                 }
                        
        //                 /*清除下发指令*/
        //                 (cmd_dev->cmd_vec).erase(cmd_it);
        //             }
        //         }
        //     }
        // }
        // /***************************指令任务**********************///备份
        // /*如果检测到 cmd_state状态为 取到命令*/
        // if(cmd_state == cmd_get){
        //     for(auto cmd_dev : cmd_task_vector){
        //         /*如果此设备的指令下传没有完成*/
        //         if(cmd_dev->flag_cmd_finished == false){
        //             /*论寻发送指令个数*/
        //             for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
        //                 /* 设置 发送内容*/
        //                 set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
        //                 /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
        //                 acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
        //                 /*重发次数*/
        //                 for(int j = 0; j < max_repeat_count; j++){
        //                     /*指令 发送指令 5A A5 01 dev.number 59 data 40*/
        //                     write_port(fd_uart1, acoustic_send_string, acoustic_data_len);
        //                     //等待条件满足  //解锁
        //                     cond_res = mainthread_acoustic_cond.wait_for(lk,chrono::seconds(acoustic_response_overtime));

        //                     /*取到回应信号*/
        //                     if(cond_res == cv_status::no_timeout){
        //                         /*取到回应*/
        //                         cout << "[thread main]指令任务:read response." << endl;
        //                         /*清除下发指令*/
        //                         (cmd_dev->cmd_vec).erase(cmd_it);
        //                         break;
        //                     }
        //                     else if(j==max_repeat_count-1){
        //                     cmd_it++;///////////////////////////////////////达到最大重发次数后还没有回应才++
        //                 }
        //                 }
        //             }
        //         }
        //     }
        // }
        /*检查指令下载状态*/
        for(auto cmd_dev : cmd_task_vector){
            if((cmd_dev->cmd_vec).empty()){
                cmd_dev->flag_cmd_finished = true;
            }
        }
        if(cmd_state == 2 && check_cmd_finished()){
            cmd_state = 3;
        }
        //sleep(MAX_MAINTHREAD_ACOUSYIC_TASK_SLEEP_SEC);
    }
    /**********************data.txt的内容填充********************/
    //文件内容示例
    //data                      -->数据标题 以下表示接收到的数据
    //1 Sampling base station   -->节点 节点属性
    //~~~~~~~~~~~(data)         -->节点数据
    //2 Monitoring base station
    //~~~~~~~~~~~(data)
    /*以上部分在接收到数据后填充了*/

    //cmd log                   -->下传指令日志
    //cmd_state:
    //1 Sampling base station   -->节点 节点属性
    //receive:2  load:1         -->接收到的指令数  已经下传的指令数
    //rest：~~~~~(cmd)          -->未下传的指令(用#分隔)
    //2 Monitoring base station
    //receive:  load:
    //rest：

    //a+ 以附加方式打开可读写的文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾后，即文件原先的内容会被保留。 
    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:Start to write datax.txt[{}].",cloud_file_path);
    SPDLOG_LOGGER_INFO(console, "[thread main]:Start to write datax.txt[{}].",cloud_file_path);

    if((fd_cloud = fopen(cloud_file_path.c_str(),"a+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:Error opening dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "[thread main]:Error opening dataconfig.txt");
        //cerr<<"[thread 1.1]:Error opening dataconfig.txt"<<endl;
    }
    string w_temp;
    //写入 cmd log
    w_temp = "cmd log\n";
    fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
    w_temp.clear();
    //写入 cmd_state
    w_temp = "cmd_state:"+ to_string(cmd_state) + "\n";
    fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
    w_temp.clear();
    //写入指令下传状态
    for(auto cmd_dev : cmd_task_vector){
        /*节点编号 节点属性*/
        w_temp.clear();
        w_temp = to_string(cmd_dev->devnum) +" "+ acousticdevice[cmd_dev->devnum-1].attribute+"\n";
        fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);

        /*receive: load:*/
        w_temp.clear();
        w_temp = "receive:" + to_string(cmd_dev->cmd_num) + " load:"+to_string(cmd_dev->cmd_num-(cmd_dev->cmd_vec).size())+"\n";
        fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);

        /*rest cmd:*/
        if((cmd_dev->cmd_vec).size()>0){
            w_temp.clear();
            w_temp = "rest cmd:" ;
            for(auto cmd_string : cmd_dev->cmd_vec){
                w_temp+=cmd_string+"\n";
            }
            fwrite(w_temp.c_str(),1,w_temp.size(),fd_cloud);
        }
    }
    fflush(fd_cloud);
    fclose(fd_cloud);
    /********************************************************/

    /*开卫星*/
    //开卫星执行一次 is_statelite_open+1 
    if(is_statelite_open == 0){
        is_statelite_open +=  open_statelite();
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:statelite open success");
        SPDLOG_LOGGER_INFO(console, "[thread main]:statelite open success");
        //printf("main:peripheral open %s success, fd = %d.\n", control_board_uart_path, fd_uart);
    }
    else{
        is_statelite_open++;
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:statelite has opened. is_statelite_open:{0:d}",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread main]:statelite has opened. is_statelite_open:{0:d}",is_statelite_open );
        
    }
    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[sem]sem_satellite waiting.....................");
    SPDLOG_LOGGER_INFO(console, "[thread main]:[sem]sem_satellite waiting.....................");
    /*获取信号量  上锁*/
    sem_wait(&sem_satellite);

    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[sem]get the sem_satellite!!!!!!!!!!!!!!!!!!!");
    SPDLOG_LOGGER_INFO(console,  "[thread main]:[sem]get the sem_satellite!!!!!!!!!!!!!!!!!!!");

    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:check cloud ssh connection. [ip:{0}] [username:{1}]",cloudserver.remote_ip,cloudserver.username);
    SPDLOG_LOGGER_INFO(console, "[thread main]:check cloud ssh connection. [ip:{0}] [username:{1}]",cloudserver.remote_ip,cloudserver.username);
    unsigned int ssh_connect_time = 0;
    while(check_ssh_connection(cloudserver.remote_ip,cloudserver.username) == false){
        ssh_connect_time += 10;
        sleep(10); //10s检测一次，直到成功
        if(ssh_connect_time%100 == 0){
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:check cloud ssh connection. [ip:{0}] [username:{1}] [time:{2}s]",cloudserver.remote_ip,cloudserver.username,ssh_connect_time);
            SPDLOG_LOGGER_INFO(console, "[thread main]:check cloud ssh connection. [ip:{0}] [username:{1}] [time:{2}s]",cloudserver.remote_ip,cloudserver.username,ssh_connect_time);
        }
        
    }
    /**/
    /*发送sendtocloude/save/datax.txt文件到云端*/
    //copydatatocloude_cmd : scp data.txt username@ip:remote_addr
    string copydatatocloude_cmd = "scp "+ cloud_file_path+" "+ \
                                        cloudserver.username + "@" + \
                                        cloudserver.remote_ip +":"+ \
                                        cloudserver.data_addr;
    popen_fp = popen(copydatatocloude_cmd.c_str(), "r");
    if(popen_fp == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
        SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
    }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
        sleep(3);
        pclose_fp=pclose(popen_fp);
        if(pclose_fp==-1){
            SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
            SPDLOG_LOGGER_ERROR(console, "pclose failed!");
        }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
        }         
    }
    /**/
    mylogger->flush();
    /*发送logs/log.log文件到云端*/
    //copydatatocloude_cmd : scp log.log username@ip:remote_addr
    string copylogtocloude_cmd = "scp "+ log_path+" "+ \
                                        cloudserver.username + "@" + \
                                        cloudserver.remote_ip +":"+ \
                                        cloudserver.log_addr;
    popen_fp = popen(copylogtocloude_cmd.c_str(), "r");
    if(popen_fp == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
        SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
    }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
        sleep(3);
        pclose_fp=pclose(popen_fp);
        if(pclose_fp==-1){
            SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
            SPDLOG_LOGGER_ERROR(console, "pclose failed!");
        }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
        }         
    }
    /*检测指令状态 判断是否要采集指令*/
    //如果还没获取指令
    if(cmd_state == cmd_no_get){
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:Check cmd.txt exsit or not.cmd_state:{}",cmd_state);
        SPDLOG_LOGGER_INFO(console, "[thread main]:Check cmd.txt exsit or not.cmd_state:{}",cmd_state);
        /*检查云服务器cmd.txt是否存在*/
        if(check_cloud_cmd_file(cloudserver.cmd_addr.substr(0,cloudserver.cmd_addr.size()-strlen("cmd.txt")-1) ,"cmd.txt",0)== true){
            /*取指令（文件获取）*/
            //copycmd_cmd : scp username@ip:remotecmd_addr cmddir
            string copycmd_cmd = "scp "+cloudserver.username + "@" + cloudserver.remote_ip +":"+\
                                        cloudserver.cmd_addr + " " + cmd_file;
            popen_fp = popen(copycmd_cmd.c_str(), "r");
            if(popen_fp == NULL){
                SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
                SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp] from cloud to get cmdfile. [{}]",copycmd_cmd);
                SPDLOG_LOGGER_INFO(console, "[thread main]:[scp] from cloud to get cmdfile. [{}]",copycmd_cmd);
                sleep(3);
                pclose_fp=pclose(popen_fp);
                if(pclose_fp==-1){
                    SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                    SPDLOG_LOGGER_ERROR(console, "pclose failed!");
                }
                else{
                    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                    SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                }         
            }
            
            /*删除云平台的文件*///////////////////////////////////测试注释掉，使用时打开
            delete_cloud_cmd_file(cloudserver.cmd_addr);
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:delete cloud cmdfile.");
            SPDLOG_LOGGER_INFO(console, "[thread main]:delete cloud cmdfile.");

            /*发送SIGUSR1信号给当前进程*/

            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[kill SIGUSR1]signal handler.");
            SPDLOG_LOGGER_INFO(console, "[thread main]:[kill SIGUSR1]signal handler.");
            kill(getpid(), SIGUSR1); 
            sleep(3);
            /***************************定时器更改采样频率任务**********************/
            /*如果检测到 cmd_state状态为 取到命令*/
            if(cmd_state == cmd_get){
                for(auto cmd_dev : cmd_task_vector){
                    /*如果此设备的指令下传没有完成*/
                    if(cmd_dev->flag_cmd_finished == false){
                        /*论寻发送指令个数*/
                        for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
                            /* 设置 发送内容*/
                            set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
                            /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
                            acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
                            /*重发次数*/
                            for(int j = 1; j < max_repeat_count; j++){
                                /*定时器复位*/
                                write_port(fd_uart4, timer_restart_cmd, timer_restart_cmd_len);
                                sleep(2);
                                /*更改定时器采样频率*/
                                cut_valid_cmd(acoustic_send_string);
                                cmd_string_vaild[cmd_string_valid_length]=0x0D;
                                cmd_string_vaild[cmd_string_valid_length+1]=0x0A;
                                write_port(fd_uart4, cmd_string_vaild, cmd_string_valid_length+2);
                                sleep(2);
                                SPDLOG_LOGGER_INFO(mylogger, "[thread main statelite]:timer changes frequency success {0:d}",cmd_string_valid_length);
                                SPDLOG_LOGGER_INFO(console, "[thread main statelite]:timer changes frequency success{0:d}",cmd_string_valid_length);
                            }
                            
                            /*清除下发指令*/
                            (cmd_dev->cmd_vec).erase(cmd_it);
                        }
                    }
                }
            }

            // /*水声通信发送指令*/
            // acoustic_cmd_task(lk);

            /*data.txt的内容填充*/
            data_file_write_contents();

            /*再发送data.txt*/
            string copydatatocloude_cmd = "scp "+ cloud_file_path+" "+\
                                            cloudserver.username + "@" +\
                                            cloudserver.remote_ip +":"+\
                                            cloudserver.data_addr;
            popen_fp = popen(copydatatocloude_cmd.c_str(), "r");
            if(popen_fp == NULL){
                SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
                SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
                SPDLOG_LOGGER_INFO(console, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
                sleep(3);
                pclose_fp=pclose(popen_fp);
                if(pclose_fp==-1){
                    SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                    SPDLOG_LOGGER_ERROR(console, "pclose failed!");
                }
                else{
                    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                    SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                }         
            }
            /**/
            mylogger->flush();
            /*发送logs/log.log文件到云端*/
            //copydatatocloude_cmd : scp log.log username@ip:remote_addr
            string copylogtocloude_cmd = "scp "+ log_path+" "+ \
                                                cloudserver.username + "@" + \
                                                cloudserver.remote_ip +":"+ \
                                                cloudserver.log_addr;
            popen_fp = popen(copylogtocloude_cmd.c_str(), "r");
            if(popen_fp == NULL){
                SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
                SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
                SPDLOG_LOGGER_INFO(console, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
                sleep(3);
                pclose_fp=pclose(popen_fp);
                if(pclose_fp==-1){
                    SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                    SPDLOG_LOGGER_ERROR(console, "pclose failed!");
                }
                else{
                    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                    SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                }         
            }    
        }
        else{
            SPDLOG_LOGGER_ERROR(mylogger, "[thread main]:could cmd.txt no exist!");
            SPDLOG_LOGGER_ERROR(console, "[thread main]:could cmd.txt no exist!"); 
            cmd_state = cmd_none;
        }
    }
    else if(cmd_state == cmd_get){
        // /*水声通信发送指令*/
        // acoustic_cmd_task(lk);

        /*data.txt的内容填充*/
        data_file_write_contents();

        /*再发送data.txt*/
        string copydatatocloude_cmd = "scp "+ cloud_file_path+" "+\
                                        cloudserver.username + "@" +\
                                        cloudserver.remote_ip +":"+\
                                        cloudserver.data_addr;
        popen_fp = popen(copydatatocloude_cmd.c_str(), "r");
        if(popen_fp == NULL){
            SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
            SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
            SPDLOG_LOGGER_INFO(console, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
            sleep(3);
            pclose_fp=pclose(popen_fp);
            if(pclose_fp==-1){
                SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                SPDLOG_LOGGER_ERROR(console, "pclose failed!");
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            }         
        }

        /**/
        mylogger->flush();
        /*发送logs/log.log文件到云端*/
        //copydatatocloude_cmd : scp log.log username@ip:remote_addr
        string copylogtocloude_cmd = "scp "+ log_path+" "+ \
                                            cloudserver.username + "@" + \
                                            cloudserver.remote_ip +":"+ \
                                            cloudserver.log_addr;
        popen_fp = popen(copylogtocloude_cmd.c_str(), "r");
        if(popen_fp == NULL){
            SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
            SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
            SPDLOG_LOGGER_INFO(console, "[thread main]:[scp] send logx.log to cloud. [{}]",copylogtocloude_cmd);
            sleep(3);
            pclose_fp=pclose(popen_fp);
            if(pclose_fp==-1){
                SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                SPDLOG_LOGGER_ERROR(console, "pclose failed!");
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            }         
        }
    }
    /*释放信号量  解锁*/
    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[sem]sem_satellite posting.....................");
    SPDLOG_LOGGER_INFO(console, "[thread main]:[sem]sem_satellite posting.....................");

    sem_post(&sem_satellite);

    SPDLOG_LOGGER_INFO(mylogger, "[thread main]:sem_post(&sem_satellite) done!!!!!!!!!!!!!!!! ");
    SPDLOG_LOGGER_INFO(console, "[thread main]:sem_post(&sem_satellite) done!!!!!!!!!!!!!!! ");


    /*关闭卫星*/
    is_statelite_open--;
    if(is_statelite_open <= 0){
        close_statelite();
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:statelite close success. [is_statelite_open:{}]",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread main]:statelite close success. [is_statelite_open:{}]",is_statelite_open);
    }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:statelite close failed. [is_statelite_open:{}]",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread main]:statelite close failed. [is_statelite_open:{}]",is_statelite_open);
    }

/////////////////////////
    // /*线程销毁标志位*/
    // flag_read_thread_exit = true;
    // pthread_cond_wait(&fatherpro_acoustic_cond,&fatherpro_acoustic_mutex);
    // pthread_mutex_unlock(&fatherpro_acoustic_mutex);//解锁

    flag_read_port_overtime=true;
    /* close port */
    close_port(fd_uart1);
    close_port(fd_uart2);
    close_port(fd_uart4);
    //阻塞等待 回收读线程
    //pthread_join(read_thread_1, NULL);
    pthread_join(read_thread_2, NULL);
    

    //退出当前线程 主线线程
    pthread_exit(NULL);
}

// template<class T>
int myclamp(const int v,const int lo,const int hi){
    return (v<lo)?lo:(hi<v)?hi:v;
}
//图像缩放函数
vector<unsigned char> resizeImage(const unsigned char* input, 
                                 int srcWidth, int srcHeight, int channels,
                                 int dstWidth, int dstHeight) {
    vector<unsigned char> output(dstWidth * dstHeight * channels);

    float xRatio = (float)srcWidth / dstWidth;
    float yRatio = (float)srcHeight / dstHeight;

    for (int y = 0; y < dstHeight; ++y) {
        for (int x = 0; x < dstWidth; ++x) {
            // 计算原始坐标（最近邻）
            int srcX = myclamp((int)(x * xRatio), 0, srcWidth - 1);
            int srcY = myclamp((int)(y * yRatio), 0, srcHeight - 1);

            // 复制像素数据
            for (int c = 0; c < channels; ++c) {
                output[(y * dstWidth + x) * channels + c] = 
                    input[(srcY * srcWidth + srcX) * channels + c];
            }
        }
    }
    return output;
}

// 将RGB数据转换为单色位图数据
vector<png_byte> convertToMono(const unsigned char* input, int width, int height, int channels) {
    vector<png_byte> output;
    int bytesPerRow = (width + 7) / 8; // 计算每行所需的字节数
    output.resize(bytesPerRow * height, 0); // 初始化为0

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // 计算当前像素的索引
            int idx = (y * width + x) * channels;
            unsigned char r = input[idx];
            unsigned char g = input[idx + 1];
            unsigned char b = input[idx + 2];

            // 计算灰度值（0-255）
            float gray = 0.299f * r + 0.587f * g + 0.114f * b;
            bool isBlack = gray < 128; // 阈值设为128

            // 计算在字节数组中的位置
            int byteIndex = x / 8;
            int bitIndex = 7 - (x % 8); // 高位在前

            if (isBlack) {
                output[y * bytesPerRow + byteIndex] |= (1 << bitIndex);
            }
        }
    }
    return output;
}
vector<png_color> generate_palette(const unsigned char* pixels, int size, int channels) {
    map<unsigned, int> color_counts;
    
    // 统计颜色频率
    for (int i = 0; i < size; i += channels) {
        unsigned key = (pixels[i]<<16) | (pixels[i+1]<<8) | pixels[i+2];
        color_counts[key]++;
    }

    // 取前16种颜色
    vector<pair<unsigned, int>> sorted(color_counts.begin(), color_counts.end());
    sort(sorted.begin(), sorted.end(), [](auto& a, auto& b) { return a.second > b.second; });

    // 生成调色板
    vector<png_color> palette;
    for (int i = 0; i < min(16, (int)sorted.size()); ++i) {
        auto& c = sorted[i].first;
        palette.push_back({(png_byte)(c>>16), (png_byte)(c>>8), (png_byte)c});
    }
    palette.resize(16, {0,0,0}); // 填充黑色

    return palette;
}

vector<unsigned char> quantize(const unsigned char* pixels, int w, int h, 
                             const vector<png_color>& palette) {
    vector<unsigned char> indices(w*h);
    
    // 找到最近颜色索引
    for (int i = 0; i < w*h*3; i += 3) {
        int best = 0, min_dist = INT_MAX;
        for (int j = 0; j < 16; ++j) {
            int dr = pixels[i] - palette[j].red;
            int dg = pixels[i+1] - palette[j].green;
            int db = pixels[i+2] - palette[j].blue;
            int dist = dr*dr + dg*dg + db*db;
            if (dist < min_dist) {
                min_dist = dist;
                best = j;
            }
        }
        indices[i/3] = best;
    }
    return indices;
}

// void write_png(const char* filename, const vector<unsigned char>& indices,
//               const vector<png_color>& palette, int w, int h) {
//     FILE* fp = fopen(filename, "wb");
//     png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, 0, 0, 0);
//     png_infop info = png_create_info_struct(png);
    
//     // 设置PNG参数
//     png_set_IHDR(png, info, w, h, 4, PNG_COLOR_TYPE_PALETTE, 
//                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
//     png_set_PLTE(png, info, palette.data(), 16);
    
//     // 打包4bpp数据
//     vector<unsigned char> packed((w*h +1)/2);
//     for (int i = 0; i < w*h; ++i) {
//         packed[i/2] |= (indices[i] << ((i%2)*4));
//     }

//     // 写入数据
//     vector<png_bytep> rows(h);
//     for (int y = 0; y < h; ++y) {
//         rows[y] = (png_byte*)packed.data() + (h-1-y)*(w/2 + w%2);
//     }
    
//     png_init_io(png, fp);
//     png_write_info(png, info);
//     png_write_image(png, rows.data());
//     png_write_end(png, 0);
    
//     png_destroy_write_struct(&png, &info);
//     fclose(fp);
// }

// // 写入PNG文件
// bool writePng(const char* filename, const vector<png_byte>& data, int width, int height) {
//     FILE* fp = fopen(filename, "wb");
//     if (!fp) return false;

//     png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
//     if (!png) {
//         fclose(fp);
//         return false;
//     }

//     png_infop info = png_create_info_struct(png);
//     if (!info) {
//         png_destroy_write_struct(&png, nullptr);
//         fclose(fp);
//         return false;
//     }

//     if (setjmp(png_jmpbuf(png))) {
//         png_destroy_write_struct(&png, &info);
//         fclose(fp);
//         return false;
//     }

//     png_init_io(png, fp);

//     // 设置PNG头部信息
//     png_set_IHDR(
//         png,
//         info,
//         width,
//         height,
//         1,                      // 位深度为1
//         PNG_COLOR_TYPE_GRAY,     // 颜色类型为灰度
//         PNG_INTERLACE_NONE,
//         PNG_COMPRESSION_TYPE_DEFAULT,
//         PNG_FILTER_TYPE_DEFAULT
//     );

//     png_write_info(png, info);

//     // 准备行指针
//     int bytesPerRow = (width + 7) / 8;
//     vector<png_bytep> rowPointers(height);
//     for (int y = 0; y < height; ++y) {
//         rowPointers[y] = (png_bytep)(data.data() + y * bytesPerRow);
//     }

//     png_write_image(png, rowPointers.data());
//     png_write_end(png, nullptr);

//     png_destroy_write_struct(&png, &info);
//     fclose(fp);
//     return true;
// }

// 直接写入原始像素数据到PNG
bool writePng(const char* filename, const unsigned char* data, int width, int height, int channels) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) return false;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        fclose(fp);
        return false;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        fclose(fp);
        return false;
    }

    if (setjmp(png_jmpbuf(png))) {
        png_destroy_write_struct(&png, &info);
        fclose(fp);
        return false;
    }

    png_init_io(png, fp);

    // 根据输入通道数设置颜色类型
    int color_type;
    if (channels == 1) color_type = PNG_COLOR_TYPE_GRAY;
    else if (channels == 3) color_type = PNG_COLOR_TYPE_RGB;
    else if (channels == 4) color_type = PNG_COLOR_TYPE_RGBA;
    else {
        cerr << "Error: Unsupported channels (" << channels << ")\n";
        return false;
    }

    png_set_IHDR(
        png,
        info,
        width,
        height,
        8,                // 位深度为8（每个通道）
        color_type,        // 自动匹配颜色类型
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT,
        PNG_FILTER_TYPE_DEFAULT
    );

    png_write_info(png, info);

    // 准备行指针（BMP数据可能为自下而上排列，需翻转）
    vector<png_bytep> rowPointers(height);
    int bytesPerRow = width * channels;
    for (int y = 0; y < height; ++y) {
        rowPointers[y] = (png_bytep)(data + y * bytesPerRow);
    }

    png_write_image(png, rowPointers.data());
    png_write_end(png, nullptr);

    png_destroy_write_struct(&png, &info);
    fclose(fp);
    return true;
}

void get_pic(string username, string ip, string pic_addr, int pic_num){
    string ssh_ls_cmd,copypicture_cmd;
    string cd_pic_cmd;
    FILE *popen_fp1;
    char tmp[512];
    set<string> pic_name;//bmp文件名
    set<string> pic_name_png;//png文件名
    /***********************ssh ls 指令 取出要采集图像的名称***************************/
    /*ssh_ls_cmd : ssh username@ip "ls pic_addr" 
    "ssh nvidia@192.168.31.220 \"ls /media/nvidia/9440-ED2D3/3DPIC/left\""
    "ssh nvidia@192.168.31.220 \"ls /media/nvidia/9440-ED2D3/3DPIC/left\""*/
    ssh_ls_cmd = "ssh " + username + "@" + ip + " \"ls " + pic_addr + "\"";
    popen_fp1 = popen(ssh_ls_cmd.c_str(), "r"); // 尝试执行 ls 命令;
    if (!popen_fp1) {
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:failed to popen");
        SPDLOG_LOGGER_ERROR(console, "[thread 2]:failed to popen");
        //cerr<<"[thread 2]:failed to popen"<<endl;
        // exit(EXIT_FAILURE);
    }
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[ssh] from local to get picfile name. [{}]",ssh_ls_cmd);
    SPDLOG_LOGGER_INFO(console, "[thread 2]:[ssh] from local to get picfile name. [{}]",ssh_ls_cmd);
    /*取pic_num个照片名*/
    while (fgets(tmp, sizeof(tmp), popen_fp1) != NULL) {
        /*只取照片*/
        if (strstr(tmp, ".bmp") != nullptr || strstr(tmp, ".jpg") != nullptr || strstr(tmp, ".png") != nullptr) {
            // if(tmp[strlen(tmp) - 1] == '\n')
            //     tmp[strlen(tmp) - 1] = '\0';

            if (tmp[strlen(tmp) - 1] == '\n' && tmp[strlen(tmp) - 2] == '\r'){
            int temp_len = strlen(tmp);
            tmp[temp_len - 1] = '\0';
            tmp[temp_len - 2] = '\0';
            }
            else if(tmp[strlen(tmp) - 1] == '\n' ){
                tmp[strlen(tmp) - 1] = '\0';
            }

            pic_name.insert(tmp);
            /*维护set只有pic_num个*/
            if(pic_name.size() > pic_num){
                pic_name.erase(pic_name.begin());
            }
        }
        
        
    }
    pclose(popen_fp1);
    /*********************************scp取照片**********************************/
    if (!pic_name.empty()) {
        /*copypicture_cmd : scp username@ip:pic1addr pic2addr ... local_addr*/
        /*scp username@ip:*/
        if(pic_name.size()==1){
            copypicture_cmd = "scp "+ username + "@" + ip +":"+pic_addr+"/";
            /*scp username@ip:pic1addr(单个文件)*/
            for(auto pic : pic_name){
                copypicture_cmd += pic + " ";
            }
            copypicture_cmd += picture_save_path; 
        }
        else{
            copypicture_cmd = "scp "+ username + "@" + ip +":"+ pic_addr+"/"+"{";
            /*scp username@ip:{pic1addr,pic2addr,...}(多文件：需要{,,,})*/
            for(auto pic : pic_name){
                copypicture_cmd += pic + ",";
            }
            copypicture_cmd.erase(copypicture_cmd.size()-1);
            copypicture_cmd += "} " + picture_save_path;
        }
        
        popen_fp1 = popen(copypicture_cmd.c_str(), "r");
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[scp] from local to get pic. [{}]",copypicture_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:[scp] from local to get pic. [{}]",copypicture_cmd);
        pclose(popen_fp1);
    }
    for(auto pic : pic_name){
        // 读取BMP文件
        //int width=0, height=0, channels=0;
        string pic_lo=picture_save_path + '/' +pic;
        // unsigned char* data = stbi_load(pic_lo.c_str(), &width, &height, &channels, 0);
        // if (!data) {
        //     cerr << "Error: Failed to load BMP file.\n";
        // }

        // // 转换为单色位图数据
        // vector<png_byte> monoData = convertToMono(data, width, height, channels);
        // stbi_image_free(data);

        string tem_png=picture_save_path + '/' +pic.substr(0,pic.find('.'))+".png";

        int targetWidth=800,targetHeight=600;
         // 加载BMP
        int srcWidth, srcHeight, channels;
        unsigned char* srcData = stbi_load(pic_lo.c_str(), &srcWidth, &srcHeight, &channels, 0);
        if (!srcData) {
            cerr << "Error: Failed to load BMP file.\n";
        }
        // 执行缩放（核心修改）
        vector<unsigned char> resizedData = resizeImage(
        srcData, srcWidth, srcHeight, channels,
        targetWidth, targetHeight
        );
        // // 处理颜色
        // auto palette = generate_palette(data, w*h*3, 3);
        // auto indices = quantize(data, w, h, palette);
        // stbi_image_free(data);
        // 写入PNG文件
        if (!writePng(tem_png.c_str(), resizedData.data(), targetWidth, targetHeight,channels)) {
            cerr << "Error: Failed to write PNG file.\n";

        }

        // // 保存PNG
        // write_png(tem_png.c_str(), indices, palette, w, h);
        string rm_cmd = "rm " + pic_lo;
        popen_fp1 = popen(rm_cmd.c_str(), "r"); 
        pclose(popen_fp1);
        pic_name_png.insert(tmp);
        /*维护set只有pic_num个*/
        if(pic_name_png.size() > pic_num){
            pic_name_png.erase(pic_name_png.begin());
        }
    }
    
   
   
}
bool write_sensor_data_by_light(const string& file,char*buf,int num){
    FILE *fp = fopen(file.c_str(),"a+");
    if(fp == nullptr)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]Cannot open file:{}",file);
        SPDLOG_LOGGER_ERROR(console, "[thread 2]Cannot open file:{}",file);
        return false;
    }
    string head  = "Bottom Station Sensor "+to_string(num);
    fwrite(head.c_str(),1,head.size(),fp);
    fwrite(buf,1,33,fp);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]Write file:{}",buf);
    SPDLOG_LOGGER_INFO(console, "[thread 2]Write file:{}",buf);
    fwrite("\n",1,strlen("\n"),fp);
    fflush(fp);
    fclose(fp);
    return true;
}
//获取今日日期字符串
std::string get_today_str() {
    time_t t = time(nullptr);
    struct tm* now = localtime(&t);
    char buf[16];
    strftime(buf, sizeof(buf), "%Y%m%d", now);
    return std::string(buf);
}

// 获取下一个 photo_record 序号
int get_next_photo_record_index(const std::string& dir_path) {
    int max_index = 0;
    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file()) {
            std::string filename = entry.path().filename().string();
            if (filename.find("photo_record_") == 0) {
                size_t pos1 = filename.find("_") + 1;
                size_t pos2 = filename.find(".txt");
                int index = std::stoi(filename.substr(pos1, pos2 - pos1));
                if (index > max_index) max_index = index;
            }
        }
    }
    return max_index + 1;
}

//获取当前照片记录
void write_photo_record() {
    std::string record_dir = "photo_record";
    std::string today = get_today_str();

    // 创建文件夹（若不存在）
    if (!fs::exists(record_dir)) {
        fs::create_directory(record_dir);
    }

    // 获取新序号文件名
    int index = get_next_photo_record_index(record_dir);
    std::string filename = record_dir + "/photo_record_" + std::to_string(index) + ".txt";

    // 写入日期
    std::ofstream fout(filename);
    fout << today << std::endl;
    fout.close();

    // 使用 SPDLOG 输出日志
    SPDLOG_LOGGER_INFO(mylogger, " success pic record: {}", filename);
    SPDLOG_LOGGER_INFO(console, " success pic record: {}", filename);
}

//判断今天是否上传过照片  遍历文件夹 判断当日日期
bool has_today_photo_uploaded(const std::string& dir_path) {
    std::string today = get_today_str();

    // 如果目录不存在，说明肯定没上传
    if (!fs::exists(dir_path)) {
        return false;
    }

    for (const auto& entry : fs::directory_iterator(dir_path)) {
        if (entry.is_regular_file()) {
            std::ifstream fin(entry.path());
            std::string line;
            if (std::getline(fin, line)) {
                if (line == today) {
                    return true;
                }
            }
        }
    }
    return false;
}


void* side_fun_thread(void* arg){
    FILE* popen_fp;
    int pclose_fp;
    struct timespec oldtime_sidethread,curtime_sidethread;//采集图片循环任务超时
    // sleep(1);
    /****************************底层基站任务初始化************************/
    vector<mybottom_base_station_task*> bottom_base_station_task_vector;  //底层采集的任务
    for(int i=0; i< bottom_decive_number; i++){
        mybottom_base_station_task* temptask = new Bottom_Base_Station_Task(); //任务建立
        temptask->dev_ip = device[i].remote_ip;  //ip初始化
        temptask->dev_username = device[i].username;  //设备用户名
        temptask->pic_addr = device[i].picture_addr;  //用户保存图片的地址数组
        temptask->pic_num = device[i].picture_number;  //采集图片的数量
        bottom_base_station_task_vector.push_back(temptask);
    }

    clock_gettime(CLOCK_REALTIME, &oldtime_sidethread);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:thread start. while overtime:{}s",MAX_CHILD_GET_PIC_WHILE_OVERTIME_SEC);
    SPDLOG_LOGGER_INFO(console, "[thread 2]:thread start. while overtime:{}s",MAX_CHILD_GET_PIC_WHILE_OVERTIME_SEC);
    /*轮询底层设备，采集图片  bottom_base_station_task_vector为空表示任务完成*/
    while(!bottom_base_station_task_vector.empty()){
        clock_gettime(CLOCK_REALTIME, &curtime_sidethread);
        /*如果超时采集图片超时*/
        if(curtime_sidethread.tv_sec-oldtime_sidethread.tv_sec >= MAX_CHILD_GET_PIC_WHILE_OVERTIME_SEC){
            break;
        }
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:start to get picture!");
        SPDLOG_LOGGER_INFO(console, "[thread 2]:start to get picture!");

        
        for(auto iter=bottom_base_station_task_vector.begin();iter!=bottom_base_station_task_vector.end();){
            /*ssh连接检测*/
            if(!check_ssh_connection( (*iter)->dev_ip , (*iter)->dev_username ) )continue;
            SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:pic dev[{}] SSH connection succeeded.", (*iter)->dev_ip);
            SPDLOG_LOGGER_INFO(console, "[thread 2]:pic dev[{}] SSH connection succeeded.", (*iter)->dev_ip);
            /*当前设备下向每个图片地址采集图片*/
            for(auto it_picaddr= ((*iter)->pic_addr).begin();it_picaddr!=((*iter)->pic_addr).end();){
                /*采集图片*/
                
                get_pic((*iter)->dev_username,(*iter)->dev_ip,(*it_picaddr),(*iter)->pic_num);
                SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:get pic [ip:{0}] [pic_addr:{1}] [pic_num:{2:d}]",(*iter)->dev_ip,(*it_picaddr),(*iter)->pic_num);
                SPDLOG_LOGGER_INFO(console, "[thread 2]:get pic [ip:{0}] [pic_addr:{1}] [pic_num:{2:d}]",(*iter)->dev_ip,(*it_picaddr),(*iter)->pic_num);
                //cout<<"[thread 2]:get pic "<<(*iter)->dev_ip<< " "<<(*it_picaddr)<<" "<<(*iter)->pic_num << '\n';              
                /*延时？*/
                sleep(3);
                /*判断图片采集到*/
                // if(check_pic_exsit()){
                //     /*图片存在，表明已经采集*/
                    /*删除当前地址*/
                    ((*iter)->pic_addr).erase(it_picaddr);
                // }
                // else{
                //     it_picaddr++;
                // }
            }
            /*检查任务地址*/
            if( ((*iter)->pic_addr).empty() ){
                bottom_base_station_task_vector.erase(iter);
            }
            else{
                iter++;
            }
        }
        /******sensor2*******/
    //创建socket
    int sockfd_2 = ::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(sockfd_2 < 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:192.168.31.11 create socket failed");
        SPDLOG_LOGGER_ERROR(console, "[thread 2]:192.168.31.11 create socket failed");
    }
    else
    {
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:192.168.31.11 create socket success");
        SPDLOG_LOGGER_INFO(console, "[thread 2]:192.168.31.11 create socket success");
    }
    int flags=fcntl(sockfd_2,F_GETFL,0);
    fcntl(sockfd_2,F_SETFL,flags | O_NONBLOCK);
    //连接服务器2
    string ip_2 ="192.168.31.11";
    int port_2 = 60001;
    struct sockaddr_in sockaddr_2;
    std::memset(&sockaddr_2,0,sizeof(sockaddr_2));
    sockaddr_2.sin_family = AF_INET;
    sockaddr_2.sin_addr.s_addr = inet_addr(ip_2.c_str());
    sockaddr_2.sin_port = htons(port_2);
    if(::connect(sockfd_2, (struct sockaddr *)&sockaddr_2,sizeof(sockaddr_2)) < 0)
    {
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:connect 192.168.31.11 failed");
        SPDLOG_LOGGER_ERROR(console, "[thread 2]:connect 192.168.31.11 failed");
    }
    else
    {
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:connect 192.168.31.11 success");
        SPDLOG_LOGGER_INFO(console, "[thread 2]:connect 192.168.31.11 success");
    }
    // //向服务端发指令
    // uint8_t sensor_cmd_2[1024];//需要指导传感器指令
    // ::send(sockfd_2,sensor_cmd_2,sizeof(sensor_cmd_2),0);
    //接收服务端数据并写入文件
    // while(1){
    //     clock_gettime(CLOCK_REALTIME, &curtime_mainthread);
    //     if(curtime_mainthread.tv_sec-oldtime_mainthread.tv_sec > MAX_FATHER_ACOUSYIC_WHILE_OVERTIME_SEC){
    //         break;  
    //     } 
    // 
    char buf_2[33];
    ::recv(sockfd_2,buf_2,sizeof(buf_2),0);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:recv:192.168.31.11 data[{}]",buf_2);
    SPDLOG_LOGGER_INFO(console, "[thread 2]:recv:192.168.31.11 data[{}]",buf_2);
    
     write_sensor_data_by_light(cloud_file_path,buf_2,2);
    //关闭socket
    ::close(sockfd_2);
    }
    /****************************socket************************/
    // /******sensor1*******/
    // //创建socket
    // int sockfd_1 = ::socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    // if(sockfd_1 < 0)
    // {
    //     SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:192.168.0.10 create socket failed");
    //     SPDLOG_LOGGER_ERROR(console, "[thread 2]:192.168.0.10 create socket failed");
    // }
    // else
    // {
    //     SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:192.168.0.10 create socket success");
    //     SPDLOG_LOGGER_INFO(console, "[thread 2]:192.168.0.10 create socket success");
    // }
    // //连接服务器1
    // string ip_1 ="192.168.0.12";
    // int port_1 = 60000;
    // struct sockaddr_in sockaddr_1;
    // std::memset(&sockaddr_1,0,sizeof(sockaddr_1));
    // sockaddr_1.sin_family = AF_INET;
    // sockaddr_1.sin_addr.s_addr = inet_addr(ip_1.c_str());
    // sockaddr_1.sin_port = htons(port_1);
    // if(::connect(sockfd_1, (struct sockaddr *)&sockaddr_1,sizeof(sockaddr_1)) < 0)
    // {
    //     SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:connect 192.168.0.10 failed");
    //     SPDLOG_LOGGER_ERROR(console, "[thread 2]:connect 192.168.0.10 failed");
    // }
    // else
    // {
    //     SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:connect 192.168.0.10 success");
    //     SPDLOG_LOGGER_INFO(console, "[thread 2]:connect 192.168.0.10 success");
    // }
    // //向服务端发指令
    // uint8_t sensor_cmd_1[1024];//需要指导传感器指令
    // ::send(sockfd_1,sensor_cmd_1,sizeof(sensor_cmd_1),0);
    // //接收服务端数据并写入文件
    // char buf_1[1024] = {0};
    // ::recv(sockfd_1,buf_1,sizeof(buf_1),0);
    // SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:recv:192.168.0.10 data[{}]",buf_1);
    // SPDLOG_LOGGER_INFO(console, "[thread 2]:recv:192.168.0.10 data[{}]",buf_1);
    // write_sensor_data_by_light(cloud_file_path,buf_1,1);
    // //关闭socket
    // ::close(sockfd_1);
    
    /*开卫星*/
    //开卫星执行一次 is_statelite_open+1 
    if(is_statelite_open == 0){
        is_statelite_open +=  open_statelite();
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:statelite open success. [is_statelite_open:{}]",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:statelite open success. [is_statelite_open:{}]",is_statelite_open);
    }
    else{
        is_statelite_open++;
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:statelite open failed. [is_statelite_open:{}]",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:statelite open failed. [is_statelite_open:{}]",is_statelite_open);
    }

    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[sem]sem_satellite waiting.....................");
    SPDLOG_LOGGER_INFO(console, "[thread 2]:[sem]sem_satellite waiting.....................");
    /*获取信号量  上锁*/
    sem_wait(&sem_satellite);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[sem]get the sem_satellite!!!!!!!!!!!!!!!!!!!");
    SPDLOG_LOGGER_INFO(console,  "[thread 2]:[sem]get the sem_satellite!!!!!!!!!!!!!!!!!!!");

    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:check cloud ssh connection. [ip:{0}] [username:{1}]",cloudserver.remote_ip,cloudserver.username);
    SPDLOG_LOGGER_INFO(console, "[thread 2]:check cloud ssh connection. [ip:{0}] [username:{1}]",cloudserver.remote_ip,cloudserver.username);

    unsigned int ssh_connect_time = 0;
    while(check_ssh_connection(cloudserver.remote_ip,cloudserver.username) == false){
        ssh_connect_time += 10;
        sleep(10); //10s检测一次，直到成功
        if(ssh_connect_time%100 == 0){
            SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:check cloud ssh connection. [ip:{0}] [username:{1}] [time:{2}s]",cloudserver.remote_ip,cloudserver.username,ssh_connect_time);
            SPDLOG_LOGGER_INFO(console, "[thread 2]:check cloud ssh connection. [ip:{0}] [username:{1}] [time:{2}s]",cloudserver.remote_ip,cloudserver.username,ssh_connect_time);
        }
        
    }
    /**/
    /*发送sendtocloude/save/datax.txt文件到云端*/
    //copydatatocloude_cmd : scp data.txt username@ip:remote_addr
    string copydatatocloude_cmd = "scp "+ cloud_file_path+" "+ \
                                        cloudserver.username + "@" + \
                                        cloudserver.remote_ip +":"+ \
                                        cloudserver.data_addr;
    popen_fp = popen(copydatatocloude_cmd.c_str(), "r");
    if(popen_fp == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
        SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
    }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread main]:[scp]Send datax.txt to cloud.[{}]",copydatatocloude_cmd);
        sleep(3);
        pclose_fp=pclose(popen_fp);
        if(pclose_fp==-1){
            SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
            SPDLOG_LOGGER_ERROR(console, "pclose failed!");
        }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
        }         
    }
    /**/
    /*发送picture/save文件夹到云端*/
    //示例：scp -r /local/folder remote.host.com:/remote/folder
    //copysavedirtocloude_cmd : scp -r picdir username@ip:remote_addr
    //scp username@ip:
    bool photo_already_uploaded = has_today_photo_uploaded("photo_record");
    if (!photo_already_uploaded) {
    write_photo_record();
    SPDLOG_LOGGER_INFO(mylogger, "first photo upload today, start uploading...");
    SPDLOG_LOGGER_INFO(console, "first photo upload today, start uploading...");
    string copysavedirtocloude_cmd = "scp -r "+ picture_save_path+" "+\
                                        cloudserver.username +"@"+ cloudserver.remote_ip +":"+\
                                        *(cloudserver.picture_addr.begin());
    popen_fp = popen(copysavedirtocloude_cmd.c_str(), "r");
    if(popen_fp == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
        SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
        }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[scp]send picture/save dir to cloud.[{}]",copysavedirtocloude_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:[scp]send picture/save dir to cloud.[{}]",copysavedirtocloude_cmd); 
        sleep(3);
        pclose_fp=pclose(popen_fp);
        if(pclose_fp==-1){
            SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
            SPDLOG_LOGGER_ERROR(console, "pclose failed!");
        }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
        }         
    }
    } else {
    SPDLOG_LOGGER_INFO(mylogger, "already uploaded photo today, skip picture upload.");
    SPDLOG_LOGGER_INFO(console, "already uploaded photo today, skip picture upload.");
}
    /**/
    mylogger->flush();
    /*发送logs/log.log文件到云端*/
    //copydatatocloude_cmd : scp log.log username@ip:remote_addr
    string copylogtocloude_cmd = "scp "+ log_path+" "+ \
                                        cloudserver.username + "@" + \
                                        cloudserver.remote_ip +":"+ \
                                        cloudserver.log_addr;
    popen_fp = popen(copylogtocloude_cmd.c_str(), "r");
    if(popen_fp == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
        SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
        }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[scp]send log.log to cloud.[{}]",copylogtocloude_cmd);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:[scp]send log.log dir to cloud.[{}]",copylogtocloude_cmd);
        sleep(3);
        pclose_fp=pclose(popen_fp);
        if(pclose_fp==-1){
            SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
            SPDLOG_LOGGER_ERROR(console, "pclose failed!");
        }
        else{
            SPDLOG_LOGGER_INFO(mylogger, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
            SPDLOG_LOGGER_INFO(console, "[thread main]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
        }         
    }
    if(cmd_state == cmd_no_get){
        /*检查云服务器cmd.txt是否存在*/
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:check cmd.txt of could exsit or not.");
        SPDLOG_LOGGER_INFO(console, "[thread 2]:check cmd.txt of could exsit or not.");
        if(check_cloud_cmd_file(cloudserver.cmd_addr.substr(0,cloudserver.cmd_addr.size()-strlen("cmd.txt")-1) ,"cmd.txt",0)== true){
            /*取指令（文件获取）*/
            //copycmd_cmd : scp username@ip:remotecmd_addr cmddir
            string copycmd_cmd = "scp "+cloudserver.username + "@" + cloudserver.remote_ip +":"+\
                                        cloudserver.cmd_addr + " " + cmd_file;
            popen_fp = popen(copycmd_cmd.c_str(), "r");
            if(popen_fp == NULL){
                SPDLOG_LOGGER_ERROR(mylogger, "popen failed!");
                SPDLOG_LOGGER_ERROR(console, "popen failed!"); 
            }
            else{
                SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[scp]from cloud to get cmd.txt.[{}]",copycmd_cmd);
                SPDLOG_LOGGER_INFO(console, "[thread 2]:[scp]from cloud to get cmd.txt.[{}]",copycmd_cmd);
                sleep(3);
                pclose_fp=pclose(popen_fp);
                if(pclose_fp==-1){
                    SPDLOG_LOGGER_ERROR(mylogger, "pclose failed!");
                    SPDLOG_LOGGER_ERROR(console, "pclose failed!");
                }
                else{
                    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                    SPDLOG_LOGGER_INFO(console, "[thread 2]:pclose(popen_fp)!!!!!!!!!!!!!!!! ");
                }         
            }
            
            /*删除云平台的文件*///////////////////////////////////测试注释掉，使用时打开
            delete_cloud_cmd_file(cloudserver.cmd_addr);
            SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:delete cloud cmdfile.");
            SPDLOG_LOGGER_INFO(console, "[thread 2]:delete cloud cmdfile.");
        }
        else{
            SPDLOG_LOGGER_ERROR(mylogger, "[thread 2]:cmd.txt no exist!");
            SPDLOG_LOGGER_ERROR(console, "[thread 2]:cmd.txt no exist!"); 
            cmd_state = cmd_none;
        }
    }
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[sem]sem_satellite posting.....................");
    SPDLOG_LOGGER_INFO(console, "[thread 2]:[sem]sem_satellite posting.....................");
    /*释放信号量  解锁*/
    sem_post(&sem_satellite);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:sem_post(&sem_satellite) done!!!!!!!!!!!!!!!! ");
    SPDLOG_LOGGER_INFO(console, "[thread 2]:sem_post(&sem_satellite) done!!!!!!!!!!!!!!! ");

    /*中断 已经取到指令cmd.txt*/
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:[kill SIGUSR1]signal handler.");
    SPDLOG_LOGGER_INFO(console, "[thread 2]:[kill SIGUSR1]signal handler.");
    kill(getpid(), SIGUSR1); 
    sleep(3);
    /***************************定时器更改采样频率任务**********************/
            /*如果检测到 cmd_state状态为 取到命令*/
            // if(cmd_state == cmd_get){
                for(auto cmd_dev : cmd_task_vector){
                    SPDLOG_LOGGER_INFO(mylogger, "1111111111111111111111");
                    SPDLOG_LOGGER_INFO(console, "1111111111111111111111");
                    /*如果此设备的指令下传没有完成*/
                    // if(cmd_dev->flag_cmd_finished == false){
                        /*论寻发送指令个数*/
                        for(auto cmd_it = (cmd_dev->cmd_vec).begin() ; cmd_it != (cmd_dev->cmd_vec).end();){
                            /* 设置 发送内容*/
                            set_acoustic_send_data(acoustic_data_function_cmdtransmit_D,cmd_dev->devnum,1,1,*cmd_it,acoustic_send_string);
                            /* 设置 接收数据解析标志 {通讯的节点编号，解析标志位}*/
                            acoustic_receiverdata_state = {cmd_dev->devnum,acoustic_receivedata_state_waitresponse};
                            SPDLOG_LOGGER_INFO(mylogger, "2222222222222222");
                            SPDLOG_LOGGER_INFO(console, "22222222222222");
                            /*重发次数*/
                            for(int j = 1; j < max_repeat_count; j++){
                                /*定时器复位*/
                                write_port(fd_uart4, timer_restart_cmd, timer_restart_cmd_len);
                                sleep(2);
                                /*更改定时器采样频率*/
                                cut_valid_cmd(acoustic_send_string);
                                cmd_string_vaild[cmd_string_valid_length]=0x0D;
                                cmd_string_vaild[cmd_string_valid_length+1]=0x0A;
                                write_port(fd_uart4, cmd_string_vaild, cmd_string_valid_length+2);
                                sleep(2);
                                SPDLOG_LOGGER_INFO(mylogger, "[thread main statelite]:timer changes frequency success {0:d}",cmd_string_valid_length);
                                SPDLOG_LOGGER_INFO(console, "[thread main statelite]:timer changes frequency success{0:d}",cmd_string_valid_length);
                            }
                            
                            /*清除下发指令*/
                            (cmd_dev->cmd_vec).erase(cmd_it);
                        }
                    // }
                }
            // }

    /*关闭卫星*/
    is_statelite_open--;
    if(is_statelite_open <= 0){
        close_statelite();
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:statelite close success. is_statelite_open:{}",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:statelite close success. is_statelite_open:{}",is_statelite_open);
    }
    else{
        SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:statelite close failed. is_statelite_open:{}",is_statelite_open);
        SPDLOG_LOGGER_INFO(console, "[thread 2]:statelite close failed. is_statelite_open:{}",is_statelite_open);
    }
    SPDLOG_LOGGER_INFO(mylogger, "[thread 2]:thread 2 task finish. now exit!");
    SPDLOG_LOGGER_INFO(console, "[thread 2]:thread 2 task finish. now exit!");
    /*退出副线程*/
    pthread_exit(NULL);
}
/*****************************************************/
/************************沉浮读线程************************/
/*****************************************************/
void* sinkfloat_read_thread(void* arg){
     //a+ 以附加方式打开可读写的文件。若文件不存在，则会建立该文件，如果文件存在，写入的数据会被加到文件尾后，即文件原先的内容会被保留。 
    if((fd_sinkfloat = fopen(sinkfloat_file_path.c_str(),"a+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening sinkfloat_dataconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening sinkfloat_dataconfig.txt");
        //cerr<<"Error opening dataconfig.txt"<<endl;
    }
    setbuf(fd_sinkfloat,NULL);
    /***************** 串口操作 *****************/
    if( (fd_uart3 = open_port(uart3_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:sinkfloat uart3 open {0} success, fd = {1:d}.\n", uart3_path, fd_uart3);
        SPDLOG_LOGGER_INFO(console, "[thread 3]:sinkfloat uart3 open {0} success, fd = {1:d}.\n", uart3_path, fd_uart3);
        //printf("thread 3:open %s success, fd = %d.\n", uart3_path, fd_uart3);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 3]:sinkfloat uart3 open {0} failed.", uart3_path);
        SPDLOG_LOGGER_ERROR(console, "[thread 3]:sinkfloat uart3 open {0} failed.", uart3_path);
    }
    /* config port */
    config_port(fd_uart3, B115200);
    /*******************************************/
    float deep=0.0,flow=0.0;
    while(1)
    {
        //读串口数据
        char sinkfloat_read_string[10];
        memset(sinkfloat_read_string,0,sizeof sinkfloat_read_string);
        read_port(fd_uart3, sinkfloat_read_string, sizeof sinkfloat_read_string, 1, 8);//8测试一下
        if (sinkfloat_read_string[strlen(sinkfloat_read_string) - 1] == '\n' && sinkfloat_read_string[strlen(sinkfloat_read_string) - 2] == '\r'){
            int temp_len = strlen(sinkfloat_read_string);
            sinkfloat_read_string[temp_len - 1] = '\0';
            sinkfloat_read_string[temp_len - 2] = '\0';
        }
        else if(sinkfloat_read_string[strlen(sinkfloat_read_string) - 1] == '\n' ){
            sinkfloat_read_string[strlen(sinkfloat_read_string) - 1] = '\0';
        }
        else if(sinkfloat_read_string[strlen(sinkfloat_read_string) - 1] == '\r' ){
            sinkfloat_read_string[strlen(sinkfloat_read_string) - 1] = '\0';
        }
        string tem = sinkfloat_read_string;
        if(tem.size()>1)
        {
        string a = tem.substr(1,tem.find('.')); 
        string b = tem.substr(tem.find('.')+1,tem.size()- tem.find('.')-1);
        if(sinkfloat_read_string[0]=='0')
        {
            deep=stof(a)+stof(b)/(pow(10,b.size())-1);
            //写入文件
            fprintf(fd_sinkfloat,"deep:%f\r\n",deep);
        }
        else if(sinkfloat_read_string[0]=='1')
        {
            flow=stof(a)+stof(b)/(pow(10,b.size())-1);
            //写入文件
            fprintf(fd_sinkfloat,"flow:%f\r\n",flow);
        }
        if(sinkfloat_read_string[0]=='3')
        {
            memset(sinkfloat_read_string,0,sizeof sinkfloat_read_string);
            tem.clear();
            fflush(fd_sinkfloat);
            break;
        }
        fflush(fd_sinkfloat);
       
        cout<<deep<<endl;
        cout<<flow<<endl;
        
        memset(sinkfloat_read_string,0,sizeof sinkfloat_read_string);
        tem.clear();
        }
    }
      /* close port */
    close_port(fd_uart3);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:sinkfloat_read_thread finish");
    SPDLOG_LOGGER_INFO(console, "[thread 3]:sinkfloat_read_thread finish");
    fflush(fd_sinkfloat);
    fclose(fd_sinkfloat);
    return NULL;
}

void uart_send_sinkfloat_once_cmd(int uart_fp,int mode,string dp,string flow){
    /*设置上浮数据格式*/
    if(mode == 2)
    {
        uint8_t send_str_float[sink_float_cmd_len];
        send_str_float[0]=0xFF;
        send_str_float[1]=0x02;
        for(int i=0;i<dp.size();i++)
        {
            send_str_float[i+2]=dp[i];
        }
        for(int i=0;i<flow.size();i++)
        {
            send_str_float[i+6]=flow[i];
        }
        // uint16_t CRC_DATA_float = crc16_cal(send_str_float,10);
        // send_str_float[10] = CRC_DATA_float>>8;
        // send_str_float[11] = CRC_DATA_float;
        send_str_float[10] = 0x40;
        for(int i=0;i<3;i++)
        {
            write_port(uart_fp, send_str_float, sizeof send_str_float);
           // usleep(1000*500);
        }

        
    } 
    /*设置下沉数据格式*/
    else if(mode == 1)
    {
        uint8_t send_str_sink[sink_float_cmd_len];
        send_str_sink[0]=0xFF;
        send_str_sink[1]=0x01;
        for(int i=0;i<dp.size();i++)
        {
            send_str_sink[i+2]=dp[i];
        }
        for(int i=0;i<flow.size();i++)
        {
            send_str_sink[i+6]=flow[i];
        }
        // uint16_t CRC_DATA_sink = crc16_cal(send_str_sink,10);
        // send_str_sink[10] = CRC_DATA_sink>>8;
        // send_str_sink[11] = CRC_DATA_sink;
        send_str_sink[10] = 0x40;
        for(int i=0;i<5;i++)
        {
            write_port(uart_fp, send_str_sink, sizeof send_str_sink);
            usleep(1000*500);
        }
            SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:uart_send_sinkfloat_once_cmd！！！！！！！！");
            SPDLOG_LOGGER_INFO(console, "[thread 3]:uart_send_sinkfloat_once_cmd！！！！！！！");
        
        
    }
}

void* sinkfloat_fun_thread(void* arg){

    sem_wait(&sem_sinkfloat_thread_open);
    pthread_t sinkfloatread_thread ;  //沉浮读线程
    /***************** 串口操作 *****************/
    if( (fd_uart3 = open_port(uart3_path)) > 0){
        SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:uart3 open {0} success, fd = {1:d}.\n", uart3_path, fd_uart3);
        SPDLOG_LOGGER_INFO(console, "[thread 3]:uart3 open {0} success, fd = {1:d}.\n", uart3_path, fd_uart3);
        //printf("thread 3:open %s success, fd = %d.\n", uart3_path, fd_uart3);
    }
    else{
        SPDLOG_LOGGER_ERROR(mylogger, "[thread 3]:uart3 open {0} failed.", uart3_path);
        SPDLOG_LOGGER_ERROR(console, "[thread 3]:uart3 open {0} failed.", uart3_path);
    }
    /* config port */
    config_port(fd_uart3, B115200);
    /*******************************************/
    // sinking_and_floating_module_mode=1;
    // sink_float_depth="0500";
    // sink_float_flow="1225";
    switch(sinking_and_floating_module_mode){
        /*无*/
        case 0:
            break;

        /*沉浮一次*/
        case 1:{
            uart_send_sinkfloat_once_cmd(fd_uart3,1,sink_float_depth,sink_float_flow);
            /* close port */
            close_port(fd_uart3);
            SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:uart_send_sinkfloat_once_cmd");
            SPDLOG_LOGGER_INFO(console, "[thread 3]:uart_send_sinkfloat_once_cmd");
            pthread_create(&sinkfloatread_thread , NULL, sinkfloat_read_thread ,(void*)"沉浮读线程");
            pthread_join(sinkfloatread_thread , NULL);
        }
            
        break;

        /*自动沉浮*/
        case 2:
            break;

        default:
            break;
    }

    //  /* close port */
    // close_port(fd_uart3);
    SPDLOG_LOGGER_INFO(mylogger, "[thread 3]:thread 3 task finish. now exit!");
    SPDLOG_LOGGER_INFO(console, "[thread 3]:thread 3 task finish. now exit!");
    /*退出沉浮线程*/
    pthread_exit(NULL);
}