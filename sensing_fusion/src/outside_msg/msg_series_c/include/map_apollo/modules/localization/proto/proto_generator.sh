#!/bin/sh

echo "************ Generating proto files (start)...\n"

cd `dirname $0`
CURR_PATH=$(pwd)
cd ../../..
WORK_PATH=$(pwd)

echo "CURR_PATH = $CURR_PATH"
echo "WORK_PATH = $WORK_PATH"

SRC_DIR=$CURR_PATH
DST_DIR=$WORK_PATH


proto_file_list=`ls $SRC_DIR/*.proto`
echo "\n>>> The proto files are: "
for file in $proto_file_list
do 
 echo $file
done

echo "\n>>> Information when generating proto files: "
for file in $proto_file_list
do 
 /usr/local/protobuf-3.9.1/bin/protoc --proto_path=$WORK_PATH --cpp_out=$DST_DIR $file
done

echo "\n************ Generating proto files (end)"

