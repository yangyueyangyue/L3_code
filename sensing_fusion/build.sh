#!usr/bin/bash

if [ -f CMakeLists.txt ];then 
     if [ -d build/ ];then 
          :
     else
          echo "[Shell] <------ The build folder doesn't exist. ------>"
          mkdir build
     fi
     cd build
     echo "[Shell] <-------------- Cmake Starts. --------------->"
     cmake ..
     echo "[Shell] <--------------- Make Starts. --------------->"
     make -j`cat /proc/cpuinfo| grep processor| wc -l`
     cd ..
else
     echo "[Shell] <----- The CMakeLists.txt doesn't exist.----->"
fi
