#!/bin/sh

echo "\033[44;37m************ Generating proto files (start)...\033[0m\n"


cd `dirname $0`
CURR_PATH=$(pwd)
#cd ../..
WORK_PATH=$(pwd)

echo "CURR_PATH = $CURR_PATH"
echo "WORK_PATH = $WORK_PATH"


echo "\nNumber of parameters : $#"
if [ $# -gt 0 ];then
  PROTOC_CMD=$1
else
  PROTOC_CMD=protoc
fi
echo "\033[36mPROTOC_CMD = $PROTOC_CMD\033[0m"


SRC_DIR=$CURR_PATH
DST_DIR=$WORK_PATH


proto_file_list=`ls $SRC_DIR/*.proto`
echo "\n>>> The proto files are: "
for file in $proto_file_list
do 
  echo $file
done

echo "\n>>> Information when generating proto files: "
echo "\033[31m"
for file in $proto_file_list
do 
  $PROTOC_CMD --proto_path=$WORK_PATH --cpp_out=$DST_DIR $file
done
echo "\033[0m"

echo "\n************ Generating proto files (end)\n\n"

