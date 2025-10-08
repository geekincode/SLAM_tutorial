/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/config.h"
#include <fstream>
#include <sstream>

namespace myslam 
{
    
bool readFileStorage(const std::string& filepath, cv::FileStorage& fs) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        std::cerr << "Cannot open file: " << filepath << std::endl;
        return false;
    }

    std::stringstream buffer;
    std::string line;

    // 检查第一行是否为 %YAML:1.0
    std::getline(file, line);
    if (line.find("%YAML:") != 0) {
        buffer << "%YAML:1.0\n";  // 注入标准头部
    }
    buffer << line << "\n";

    // 写入剩余内容
    buffer << file.rdbuf();
    file.close();

    // 从字符串流打开 FileStorage
    fs.open(buffer.str(), cv::FileStorage::READ | cv::FileStorage::MEMORY);
    return fs.isOpened();
}
    
void Config::setParameterFile( const std::string& filename )
{
    if ( config_ == nullptr )
        config_ = shared_ptr<Config>(new Config);
    
    // 使用兼容性函数读取YAML文件
    if (!readFileStorage(filename, config_->file_)) {
        std::cerr << "parameter file " << filename << " does not exist or cannot be opened." << std::endl;
        return;
    }
}

Config::~Config()
{
    if ( file_.isOpened() )
        file_.release();
}

shared_ptr<Config> Config::config_ = nullptr;

}