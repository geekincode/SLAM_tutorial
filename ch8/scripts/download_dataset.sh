#!/bin/bash

# Download script for rgbd_dataset_freiburg3_sitting_static dataset

DATASET_NAME="rgbd_dataset_freiburg3_sitting_static"
DOWNLOAD_URL="https://vision.in.tum.de/rgbd/dataset/freiburg3/${DATASET_NAME}.tgz"
DATASET_DIR="${HOME}/SLAM/Book/slamtest/ch8/${DATASET_NAME}"

echo "Checking if dataset already exists..."
if [ -d "${DATASET_DIR}" ]; then
    echo "Dataset directory already exists at ${DATASET_DIR}"
    exit 0
fi

echo "Creating dataset directory..."
mkdir -p "${DATASET_DIR}"

echo "Downloading ${DATASET_NAME} dataset..."
wget -O "/tmp/${DATASET_NAME}.tgz" "${DOWNLOAD_URL}"

if [ $? -ne 0 ]; then
    echo "Failed to download dataset"
    exit 1
fi

echo "Extracting dataset..."
tar -xzf "/tmp/${DATASET_NAME}.tgz" -C "${HOME}/SLAM/Book/slamtest/ch8/"

echo "Cleaning up temporary files..."
rm "/tmp/${DATASET_NAME}.tgz"

echo "Dataset downloaded and extracted successfully!"