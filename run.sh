#!/bin/bash

for filename in *.txt
do
    g++ -std=c++11 bnb_new_3.cpp -o bnb_new_3 && ./bnb_new_3 "$filename"
done

