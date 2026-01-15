/**
 * @file config.cpp
 * @brief 配置文件管理类的实现
 * 
 * 实现YAML配置文件的读取和参数获取功能
 * 包含YAML文件格式兼容性处理
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#include "myslam/config.h"
#include <fstream>
#include <sstream>

namespace myslam 
{

/**
 * @brief 兼容性地读取YAML文件
 * 
 * 由于OpenCV的FileStorage对YAML格式有严格要求，
 * 此函数在必要时自动添加YAML头部声明 "%YAML:1.0"
 * 
 * @param filepath YAML文件路径
 * @param fs 输出的FileStorage对象
 * @return 是否成功打开文件
 */
bool readFileStorage(const std::string& filepath, cv::FileStorage& fs) {
    // 打开文件
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filepath << std::endl;
        return false;
    }

    std::stringstream buffer;
    std::string line;

    // 检查第一行是否为 %YAML:1.0 格式头
    std::getline(file, line);
    if (line.find("%YAML:") != 0) {
        // 如果没有YAML头部，自动注入标准头部
        buffer << "%YAML:1.0\n";
    }
    buffer << line << "\n";

    // 读取剩余内容
    buffer << file.rdbuf();
    file.close();

    // 从内存字符串流打开FileStorage
    // cv::FileStorage::MEMORY 表示从内存而非文件读取
    fs.open(buffer.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    return fs.isOpened();
}
    
/**
 * @brief 设置配置文件路径
 * 
 * 使用单例模式：首次调用时创建Config实例，后续调用重用该实例
 * 
 * @param filename 配置文件的完整路径
 */
void Config::setParameterFile( const std::string& filename )
{
    // 如果单例尚未创建，则创建之
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);
    
    // 使用兼容性函数读取YAML文件
    if (!readFileStorage(filename, config_->file_)) {
        std::cerr << "parameter file " << filename << " does not exist or cannot be opened." << std::endl;
        return;
    }
}

/**
 * @brief 析构函数
 * 
 * 释放FileStorage资源
 */
Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

// 静态成员初始化：单例指针初始为空
shared_ptr<Config> Config::config_ = nullptr;

}