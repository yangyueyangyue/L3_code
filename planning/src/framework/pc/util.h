/* Copyright 2018,2019 Kotei Co., Ltd.
 ******************************************************************************
 ** Some common operator
 ******************************************************************************
 *
 *  Some common operator
 *
 *  @file       util.h
 *
 *  @author     kotei
 *  @date       2018.11.22
 *  @version    001 2018.11.22  新规作成
 *              002 2019.05.21  将使用ROS函数获取耗时的方法修改为
 *                              使用linux系统函数的方式
 *
 ******************************************************************************
 *  All Rights Reserved. Copyright(C) 2018,2019 Kotei Co., Ltd.
 ******************************************************************************/

/**
 * @file
 * @brief Some util functions.
 */

#ifndef PHOENIX_FRAMEWORK_UTIL_H_
#define PHOENIX_FRAMEWORK_UTIL_H_

#include <fcntl.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "ros/ros.h"
#include "boost/timer.hpp"

#include "google/protobuf/io/zero_copy_stream_impl.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"

#include "geometry/vec2d.h"
#include "modules/common/proto/geometry.pb.h"
#include "string_util.h"


/**
 * @namespace phoenix::common
 * @brief phoenix::common
 */
namespace phoenix {
namespace framework {


template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message, int file_descriptor) {
  using google::protobuf::io::ZeroCopyOutputStream;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::TextFormat;
  if (file_descriptor < 0) {
    LOG_ERR << "Invalid file descriptor.";
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        ascii representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
template <typename MessageType>
bool SetProtoToASCIIFile(const MessageType &message,
                         const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    LOG_ERR << "Unable to open file " << file_name.c_str() << " to write.";
    return false;
  }
  return SetProtoToASCIIFile(message, fd);
}

/**
 * @brief Parses the content of the file specified by the file_name as ascii
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromASCIIFile(const std::string &file_name, MessageType *message) {
  using google::protobuf::io::ZeroCopyInputStream;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::TextFormat;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    LOG_ERR << "Failed to open file " << file_name.c_str() << " in text mode.";
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    LOG_ERR << "Failed to parse file " << file_name.c_str() << " as text proto.";
  }
  delete input;
  close(file_descriptor);
  return success;
}

/**
 * @brief Sets the content of the file specified by the file_name to be the
 *        binary representation of the input protobuf.
 * @param message The proto to output to the specified file.
 * @param file_name The name of the target file to set the content.
 * @return If the action is successful.
 */
template <typename MessageType>
bool SetProtoToBinaryFile(const MessageType &message,
                          const std::string &file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  return message.SerializeToOstream(&output);
}

/**
 * @brief Parses the content of the file specified by the file_name as binary
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromBinaryFile(const std::string &file_name,
                            MessageType *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    LOG_ERR << "Failed to open file " << file_name.c_str() << " in binary mode.";
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    LOG_ERR << "Failed to parse file " << file_name.c_str() << " as binary proto.";
    return false;
  }
  return true;
}

/**
 * @brief Parses the content of the file specified by the file_name as a
 *        representation of protobufs, and merges the parsed content to the
 *        proto.
 * @param file_name The name of the file to parse whose content.
 * @param message The proto to carry the parsed content in the specified file.
 * @return If the action is successful.
 */
template <typename MessageType>
bool GetProtoFromFile(const std::string &file_name, MessageType *message) {
  // Try the binary parser first if it's much likely a binary proto.
  if (EndWith(file_name, ".bin")) {
    return GetProtoFromBinaryFile(file_name, message) ||
        GetProtoFromASCIIFile(file_name, message);
  }

  return GetProtoFromASCIIFile(file_name, message) ||
      GetProtoFromBinaryFile(file_name, message);
}


class Dormancy {
 public:
  explicit Dormancy(int ms) : rate_(ms > 0 ? 1000/ms : 100) {}
  void Sleep() { rate_.sleep(); }
 private:
  ros::Rate rate_;
};


}  // namespace framework
}  // namespace phoenix


#endif  // PHOENIX_FRAMEWORK_UTIL_H_
