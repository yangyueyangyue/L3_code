#!/bin/sh


cd `dirname $0`
CURR_PATH=$(pwd)
echo "CURR_PATH = $CURR_PATH"

PROTOC_CMD=/usr/local/protobuf-3.9.1/bin/protoc
echo "\033[36mPROTOC_CMD = $PROTOC_CMD\033[0m"


cd $CURR_PATH/include
./proto_generator.sh $PROTOC_CMD

# Apollo map (begin)
cd $CURR_PATH/include/map_apollo/modules/common/proto
./proto_generator.sh $PROTOC_CMD

cd $CURR_PATH/include/map_apollo/modules/localization/proto
./proto_generator.sh $PROTOC_CMD

cd $CURR_PATH/include/map_apollo/modules/map/proto
./proto_generator.sh $PROTOC_CMD

cd $CURR_PATH/include/map_apollo/modules/routing/proto
./proto_generator.sh $PROTOC_CMD
# Apollo map (end)


