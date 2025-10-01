#!/usr/bin/env python3
import yaml
import os

# 获取脚本所在目录
script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, '..', 'datasets', 'config', 'url.yaml')

with open(config_path, 'r', encoding='utf-8') as file:
    config = yaml.safe_load(file)


print(f"{config['datasets'][config['select'][0]][0]}")      # DATASET_NAME
print(f"{config['datasets'][config['select'][0]][1]}")      # DATASET_URL
