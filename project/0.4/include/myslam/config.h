/**
 * @file config.h
 * @brief 配置文件管理类 - 使用单例模式管理YAML配置文件
 * 
 * 本文件定义了一个配置管理类，用于从YAML文件中读取系统参数。
 * 采用单例模式确保全局只有一个配置实例。
 * 
 * 使用示例：
 *   Config::setParameterFile("config.yaml");
 *   float fx = Config::get<float>("camera.fx");
 * 
 * Copyright (C) 2016  <copyright holder> <email>
 * This program is free software under GNU General Public License.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include "myslam/common_include.h" 
#include <opencv2/opencv.hpp>

namespace myslam 
{

/**
 * @brief 工具函数：兼容性地读取YAML文件
 * 
 * 由于不同版本的YAML文件格式可能不同，此函数确保能够正确读取
 * 
 * @param filepath YAML文件路径
 * @param fs 输出的FileStorage对象
 * @return 是否成功打开文件
 */
bool readFileStorage(const std::string& filepath, cv::FileStorage& fs);

/**
 * @class Config
 * @brief 配置文件管理类（单例模式）
 * 
 * 该类使用单例模式，在整个程序中只存在一个实例，
 * 负责管理所有系统配置参数的读取。
 */
class Config
{
private:
    static std::shared_ptr<Config> config_;   // 静态单例指针
    cv::FileStorage file_;                     // OpenCV文件存储对象，用于读取YAML
    
    /**
     * @brief 私有构造函数，确保单例模式
     */
    Config () {}
    
public:
    /**
     * @brief 析构函数，释放文件资源
     */
    ~Config();
    
    /**
     * @brief 设置配置文件路径
     * @param filename 配置文件的完整路径
     * 
     * 调用此函数后，可以使用get()方法获取配置参数
     */
    static void setParameterFile( const std::string& filename ); 
    
    /**
     * @brief 获取配置参数值（模板函数）
     * @tparam T 参数类型（int, float, double, string等）
     * @param key 参数名称（YAML中的键）
     * @return 参数值
     * 
     * 使用示例：
     *   int num = Config::get<int>("number_of_features");
     *   float fx = Config::get<float>("camera.fx");
     */
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::config_->file_[key] );
    }
};
}

#endif // CONFIG_H