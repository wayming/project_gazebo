#!/bin/bash

num=$1
./stop.bash
for i in $(seq 1 $num)
do
  ./stop.bash "drone"$i
done