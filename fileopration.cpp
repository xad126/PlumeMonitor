#include "fileopration.h"
#include "config.h"
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>


using namespace std;

/**
* @brief 获取设备属性
* @brief 配置了cloudserver(云服务器)/picture_save_path(图片保存路径)/device(底层基站摄像机设备)
* @param dev：接入设备属性
* @param cloud:接入云平台属性
* @return 返回设备数 正常获取，0 获取失败
**/
int get_config(const string& dev,const string& cloud){
    FILE* pic_fp = nullptr;
    int picture_save_number;
    // DIR* pic_dir = nullptr;
    /***************************获取cloud配置**************************/
    FILE* input_fp = nullptr ;
    char sin_char[200] = {0}; //读行缓存
    int device_number = 0; //读取设备数缓存

    if ((input_fp = fopen(cloud.c_str(), "r")) == nullptr) {
        SPDLOG_LOGGER_ERROR(mylogger, "{} no find!",cloud);
        SPDLOG_LOGGER_ERROR(console, "{} no find!",cloud);
        //cerr << cloud<<" no find!" << endl;
        return 0;
    }

    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, input_fp) != nullptr) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
        string sin = sin_char;
        if (sin.find(':') == -1) {
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "username") {
            cloudserver.username = b;
            continue;
        }
        if (a == "remote_ip") {
            cloudserver.remote_ip = b;
            continue;
        }
        if (a == "picture_addr") {
            cloudserver.picture_addr.push_back(b) ;
            continue;
        }
        if (a == "cmd_addr") {
            cloudserver.cmd_addr = b;
            continue;
        }
        if (a == "data_addr") {
            cloudserver.data_addr = b;
            continue;
        }
        if (a == "log_addr") {
            cloudserver.log_addr = b;
            continue;
        }
        
    }
    fclose(input_fp);
    
    /***************************pictureconfig配置**************************/
    /*检测文件名*/
    if((pic_fp = fopen(picture_config_file,"r+")) == NULL){
        SPDLOG_LOGGER_ERROR(mylogger, "Error opening pictureconfig.txt");
        SPDLOG_LOGGER_ERROR(console, "Error opening pictureconfig.txt");
        //cerr<<"Error opening pictureconfig.txt"<<endl;
    }
    /*读出每行数据*/
    while (fgets(sin_char, sizeof sin_char, pic_fp) != nullptr) {
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
            picture_save_number = stoi(b)+1;
            continue;
        }
    }
    picture_save_path += to_string(picture_save_number);
    string write_temp = "number:" + to_string(picture_save_number);
    fseek(pic_fp,0,SEEK_SET);
    fwrite(write_temp.c_str(),1,write_temp.size(),pic_fp);
    fclose(pic_fp);
    /****************************创建图片保存文件****************************/

    const char* path = picture_save_path.c_str();
    mode_t mode = 0755; // 设置权限为rwxr-xr-x
 
    int result = mkdir(path, mode);
    if (result == 0) {
        SPDLOG_LOGGER_INFO(mylogger, "Folder created successfully.");
        SPDLOG_LOGGER_INFO(console, "Folder created successfully.");
        //cout << "Folder created successfully." <<endl;
    } else {
        SPDLOG_LOGGER_ERROR(mylogger, "Error creating folder.");
        SPDLOG_LOGGER_ERROR(console, "Error creating folder.");
        //cerr << "Error creating folder." << endl;
    }

    /***************************获取dev配置**************************/
    if ((input_fp = fopen(dev.c_str(), "r")) == nullptr) {
        SPDLOG_LOGGER_ERROR(mylogger, "{} no find!",dev);
        SPDLOG_LOGGER_ERROR(console, "{} no find!",dev);
        //cerr << dev << " no find!" << endl;
        return 0;
    }
     /*检查第一行 设备数量*/
    if (fgets(sin_char, sizeof sin_char, input_fp) != nullptr) {
        string sin = sin_char;
        if (sin.find(':') == -1) {
            SPDLOG_LOGGER_ERROR(mylogger, "{}:have not device_number!",dev);
            SPDLOG_LOGGER_ERROR(console, "{}:have not device_number!",dev);
            //cerr << dev << ": no find!" << endl;
            return 0;
        }
        string a = sin.substr(0, sin.find(':'));
        string b = sin.substr(sin.find(':') + 1, sin.size() - sin.find(':') - 1);
        /*如果存在这个属性*/
        if (a == "device_number") {
            device_number = stoi(b);
        }
    }
    else {
        SPDLOG_LOGGER_ERROR(mylogger, "{} is empty!",dev);
        SPDLOG_LOGGER_ERROR(console, "{} is empty!",dev);
        //cerr << dev << " is empty!" << endl;
        return 0;
    }
    
    /*获取所有设备属性*/
    int temp_n = -1;
    /*可读出数据，同时不超过最大设备数*/
    while (fgets(sin_char, sizeof sin_char, input_fp) != nullptr &&\
            temp_n < MAX_decive_number &&\
            temp_n < device_number) {
        if (sin_char[strlen(sin_char) - 1] == '\n')
            sin_char[strlen(sin_char) - 1] = '\0';
        string sin = sin_char;
        /*---为每个设备的信息分界线*/
        if (sin == "---") temp_n++;
        if (sin.find(':') == -1) {
            //sin.clear();
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "username") {
            device[temp_n].username = b;
            continue;
        }
        if (a == "remote_ip") {
            device[temp_n].remote_ip = b;
            continue;
        }
        if (a == "picture_addr") {
            /*解析路径 包含#*/
            while(b.find("#")!= -1){
                device[temp_n].picture_addr.push_back(b.substr(0,b.find('#')));
                b = b.substr(b.find('#')+1,b.size()- b.find('#')-1);
            }
            device[temp_n].picture_addr.push_back(b);     
            continue;
        }
        if (a == "picture_number") {
            device[temp_n].picture_number =  stoi(b);
            continue;
        }
    }
    fclose(input_fp);
    return device_number;
}
/**
* @brief 获取声学设备属性
* @param acousticdev：接入设备属性
* @return 返回水声设备数 正常获取，0 获取失败
**/
int get_acoustic_config(string acousticdev){
    FILE *input_fp;
    char sin_char[200] = {0};
    int acousticdevice_number;
    /***************************获取acousticdev配置**************************/
    if ((input_fp = fopen(acousticdev.c_str(), "r")) == nullptr) {
        SPDLOG_LOGGER_ERROR(mylogger, "{} no find!",acousticdev);
        SPDLOG_LOGGER_ERROR(console, "{} no find!",acousticdev);
        //cerr << acousticdev << " no find!" << endl;
        return 0;
    }
     /*检查第一行 设备数量*/
    if (fgets(sin_char, sizeof sin_char, input_fp) != nullptr) {
        string sin = sin_char;
        if (sin.find(':') == -1) {
            SPDLOG_LOGGER_ERROR(mylogger, "{}:have not device_number!",acousticdev);
            SPDLOG_LOGGER_ERROR(console, "{}:have not device_number!",acousticdev);
            //cerr << acousticdev << ": have not device_number!" << endl;
            return 0;
        }
        string a = sin.substr(0, sin.find(':'));
        string b = sin.substr(sin.find(':') + 1, sin.size() - sin.find(':') - 1);
        /*如果存在这个属性*/
        if (a == "device_number") {
            acousticdevice_number = stoi(b);
        }
    }
    else {
        SPDLOG_LOGGER_ERROR(mylogger, "{} is empty!",acousticdev);
        SPDLOG_LOGGER_ERROR(console, "{} is empty!",acousticdev);
        //cerr << acousticdev << " is empty!" << endl;
        return 0;
    }
    
    /*获取所有设备属性*/
    int temp_n = -1;
    /*可读出数据，同时不超过最大设备数*/
    while (fgets(sin_char, sizeof sin_char, input_fp) != nullptr && temp_n < MAX_underwater_acoustic_communication) {
        if (sin_char[strlen(sin_char) - 1] == '\n' && sin_char[strlen(sin_char) - 2] == '\r'){
            int temp_len = strlen(sin_char);
            sin_char[temp_len - 1] = '\0';
            sin_char[temp_len - 2] = '\0';
        }
        else if(sin_char[strlen(sin_char) - 1] == '\n' ){
            sin_char[strlen(sin_char) - 1] = '\0';
        }
        string sin = sin_char;
        /*---为每个设备的信息分界线*/
        if (sin == "---") temp_n++;
        if (sin.find(':') == -1) {
            //sin.clear();
            continue; //没有：就跳过
        } 
        string a = sin.substr(0,sin.find(':')); 
        string b = sin.substr(sin.find(':')+1,sin.size()- sin.find(':')-1);
        /*如果存在这个属性*/
        if (a == "number") {
            acousticdevice[temp_n].number = stoi(b);
            continue;
        }
        if (a == "attribute") {
            acousticdevice[temp_n].attribute = b;
            continue;
        }
    }
    fclose(input_fp);
    return acousticdevice_number;
    
}

