#pragma once

#include <string>
#include <iostream>
#include <string.h>

#define dev_file "configfile/deviceconfig.txt"  //接入设备配置
#define cloud_file "configfile/cloudserverconfig.txt"  //云服务配置
#define acoustic_file "configfile/acousticdevice.txt"  //声学设备配置
using namespace std;

int get_config(const string& dev,const string& cloud);
int get_acoustic_config(string acousticdev);