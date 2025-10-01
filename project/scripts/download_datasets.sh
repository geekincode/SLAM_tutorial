#!/bin/bash

# Download script for datasets
# Using parse_yaml.sh to get download URLs from config file

# 获取脚本所在目录
script_dir=$(dirname "$0")

YAML_FILE="$script_dir/../datasets/config/url.yaml"
echo $script_dir

echo "正在解析YAML文件: $YAML_FILE"
echo "----------------------------------------"

# 执行Python脚本并捕获输出
echo "Executing Python script and capturing output..."
# 将输出读入数组，第一行是数据集名称，第二行是数据集URL
mapfile -t yaml_output < <(python3 "$script_dir/parse_yaml.py" "$YAML_FILE")
DATASET_NAME="${yaml_output[0]}"
DOWNLOAD_URL="${yaml_output[1]}"

echo "Dataset Name is: $DATASET_NAME"
echo "Dataset URL is: $DOWNLOAD_URL"

if [ -z "$DOWNLOAD_URL" ]; then
    echo "错误: 无法从配置文件中获取下载链接"
    exit 1
fi

DATASET_DIR="${HOME}/SLAM/Book/slamtest/project/datasets/${DATASET_NAME}"

echo "Checking if dataset already exists..."
if [ -d "${DATASET_DIR}" ]; then
    echo "Dataset directory already exists at ${DATASET_DIR}"
    exit 0
fi

echo "Creating dataset directory..."
mkdir -p "${DATASET_DIR}"

echo "Downloading ${DATASET_NAME} dataset..."
echo "URL: ${DOWNLOAD_URL}"
wget -O "/tmp/${DATASET_NAME}.tgz" "${DOWNLOAD_URL}"

if [ $? -ne 0 ]; then
    echo "Failed to download dataset"
    exit 1
fi

echo "Extracting dataset..."
tar -xzf "/tmp/${DATASET_NAME}.tgz" -C "${HOME}/SLAM/Book/slamtest/project/datasets/"

echo "Cleaning up temporary files..."
rm "/tmp/${DATASET_NAME}.tgz"

echo "Dataset downloaded and extracted successfully!"