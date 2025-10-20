#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
This script associates two trajectory files according to their timestamps.
It also removes consecutive entries with identical timestamps.
"""

import argparse
import sys
import os
import numpy


def read_file_list(filename):
    """
    Reads a trajectory from a text file.
    
    File format:
    The file format is "stamp x y z qx qy qz qw", where stamp denotes the
    time stamp (to which the poses are aligned to) and x y z qx qy qz qw 
    is the corresponding pose data.
    
    Parameters:
    filename -- the filename of the file that contains the trajectory
    
    Return value:
    Returns a dictionary of timestamp->pose
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n") 
    list = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line)>0 and line[0]!="#"]
    list = [(float(l[0]),l[1:]) for l in list if len(l)>1]
    return dict(list)

def associate(first_list, second_list, offset, max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim 
    to find the closest match for every input tuple.
    
    Parameters:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation

    Return value:
    List of matched tuples ((stamp1,data1),(stamp2,data2))
    """
    first_keys = list(first_list.keys())
    second_keys = list(second_list.keys())
    potential_matches = [(abs(a - (b + offset)), a, b) 
                         for a in first_keys 
                         for b in second_keys 
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
    
    matches.sort()
    return matches

def filter_consecutive_same_stamps(associations):
    """
    Filter out consecutive entries with identical timestamps.
    
    Parameters:
    associations -- list of tuples containing associated timestamps
    
    Return value:
    Filtered list with consecutive duplicates removed
    """
    if not associations:
        return associations
        
    filtered_associations = [associations[0]]
    for i in range(1, len(associations)):
        if associations[i][0] != associations[i-1][0] or associations[i][1] != associations[i-1][1]:
            filtered_associations.append(associations[i])
            
    return filtered_associations

if __name__ == '__main__':    
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('first_file', help='first text file (format: timestamp data)')
    parser.add_argument('second_file', help='second text file (format: timestamp data)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--filter_duplicates', help='filter out consecutive entries with identical timestamps (default: disabled)', action='store_true')
    parser.add_argument('--output_file', help='output file name (default: associated.txt)', default='associated.txt')
    
    args = parser.parse_args()

    first_list = read_file_list(args.first_file)
    second_list = read_file_list(args.second_file)

    matches = associate(first_list, second_list,float(args.offset),float(args.max_difference)) 
    
    if args.filter_duplicates:
        matches = filter_consecutive_same_stamps(matches)
    
    # 获取脚本所在目录
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 构建输出文件的完整路径
    output_path = os.path.join(script_dir, args.output_file)
    
    # 将结果写入文件而不是标准输出
    with open(output_path, 'w') as f:
        for a,b in matches:
            f.write("%f %s %f %s\n"%(a," ".join(first_list[a]),b," ".join(second_list[b])))
    
    print("关联结果已保存到 %s" % output_path)