#!/bin/bash

# 遍历当前目录下的所有 txt 文件
for filename in *.txt
do
    # 遍历预定的一系列超时时间
    # 执行命令并将文件名以及超时时间作为参数传递给 C++ 程序
    g++ -std=c++11 bnb_running_1.cpp -o bnb_running_1 && ./bnb_running_1 "$filename"
done

