#!/bin/bash

for filename in *.txt
do
    g++ -std=c++11 bnb.cpp -o bnb && ./bnb "$filename"
done

