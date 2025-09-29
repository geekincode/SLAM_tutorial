# RGB-D Dataset Freiburg 3 Sitting Static

This directory contains the TUM RGB-D dataset "freiburg3_sitting_static". Due to the large size of the dataset, the actual image files are not included in the Git repository.

## Dataset Information

- Dataset name: freiburg3_sitting_static
- Source: [TUM RGB-D Dataset](https://vision.in.tum.de/data/datasets/rgbd-dataset)
- Sequence type: Hand-held SLAM sequence
- Description: Person sitting in office environment, camera observing the person from different viewpoints

## How to Download the Dataset

To download and extract the dataset, run the provided script:

```bash
./scripts/download_dataset.sh
```

Alternatively, you can manually download the dataset from:
https://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_sitting_static.tgz

## Directory Structure

After downloading, the dataset will contain the following files and directories:

- `rgb/` - RGB images (707 images)
- `depth/` - Depth images (686 images)
- `rgb.txt` - Timestamps for RGB images
- `depth.txt` - Timestamps for depth images
- `accelerometer.txt` - Accelerometer readings
- `groundtruth.txt` - Ground truth trajectory
- `associate.py` - Script to associate RGB and depth images by timestamp
- `associate.txt` - Associated RGB and depth image timestamps

## Usage

To associate RGB and depth images by timestamp, use the provided `associate.py` script:

```bash
python3 associate.py rgb.txt depth.txt > associated.txt
```

This will generate a file with matched RGB and depth image timestamps that can be used for further processing.

