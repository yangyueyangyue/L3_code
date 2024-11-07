//
#include "glog/logging.h"
#if (ENABLE_GTEST)
#include "gtest/gtest.h"
#endif

#include "utils/serialization_utils.h"


#if (ENABLE_GTEST)

namespace phoenix {
namespace common {


TEST(SerializationUtils, Case_Uint8_01) {
  Uint8_t data = 0x86;
  Uint8_t decoded_data = 0;
  Uint8_t data_array[3] = { 0x86, 0x56, 0x36 };
  Uint8_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedUint8Array(&data, 1);
  Int32_t encoded_size = EncodeUint8Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeUint8Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 1);
  EXPECT_EQ(encoded_size, 1);
  EXPECT_EQ(decoded_size, 1);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedUint8Array(data_array, 3);
  encoded_size = EncodeUint8Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeUint8Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 3);
  EXPECT_EQ(encoded_size, 3);
  EXPECT_EQ(decoded_size, 3);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Int8_01) {
  Int8_t data = 0x86;
  Int8_t decoded_data = 0;
  Int8_t data_array[3] = { -19, 0x56, 0x36 };
  Int8_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedInt8Array(&data, 1);
  Int32_t encoded_size = EncodeInt8Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeInt8Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 1);
  EXPECT_EQ(encoded_size, 1);
  EXPECT_EQ(decoded_size, 1);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedInt8Array(data_array, 3);
  encoded_size = EncodeInt8Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeInt8Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 3);
  EXPECT_EQ(encoded_size, 3);
  EXPECT_EQ(decoded_size, 3);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Int16_01) {
  Int16_t data = 3000;
  Int16_t decoded_data = 0;
  Int16_t data_array[3] = { -2999, 2000, 500 };
  Int16_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedInt16Array(&data, 1);
  Int32_t encoded_size = EncodeInt16Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeInt16Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 2);
  EXPECT_EQ(encoded_size, 2);
  EXPECT_EQ(decoded_size, 2);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedInt16Array(data_array, 3);
  encoded_size = EncodeInt16Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeInt16Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 6);
  EXPECT_EQ(encoded_size, 6);
  EXPECT_EQ(decoded_size, 6);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Uint16_01) {
  Uint16_t data = 6000;
  Uint16_t decoded_data = 0;
  Uint16_t data_array[3] = { 6001, 3000, 100 };
  Uint16_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedUint16Array(&data, 1);
  Int32_t encoded_size = EncodeUint16Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeUint16Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 2);
  EXPECT_EQ(encoded_size, 2);
  EXPECT_EQ(decoded_size, 2);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedUint16Array(data_array, 3);
  encoded_size = EncodeUint16Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeUint16Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 6);
  EXPECT_EQ(encoded_size, 6);
  EXPECT_EQ(decoded_size, 6);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Int32_01) {
  Int32_t data = 300000;
  Int32_t decoded_data = 0;
  Int32_t data_array[3] = { -2999000, 2000000, 500000 };
  Int32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedInt32Array(&data, 1);
  Int32_t encoded_size = EncodeInt32Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeInt32Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedInt32Array(data_array, 3);
  encoded_size = EncodeInt32Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeInt32Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Uint32_01) {
  Uint32_t data = 6000000;
  Uint32_t decoded_data = 0;
  Uint32_t data_array[3] = { 69990000, 30000001, 500002 };
  Uint32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedUint32Array(&data, 1);
  Int32_t encoded_size = EncodeUint32Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeUint32Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedUint32Array(data_array, 3);
  encoded_size = EncodeUint32Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeUint32Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Int64_01) {
  Int64_t data = -699990000008;
  Int64_t decoded_data = 0;
  Int64_t data_array[3] = { -6999228369, 36921548962321, -89210256441 };
  Int64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedInt64Array(&data, 1);
  Int32_t encoded_size = EncodeInt64Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeInt64Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedInt64Array(data_array, 3);
  encoded_size = EncodeInt64Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeInt64Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Uint64_01) {
  Uint64_t data = 7699990000008;
  Uint64_t decoded_data = 0;
  Uint64_t data_array[3] = { 86999228369, 936921548962321, 689210256441 };
  Uint64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedUint64Array(&data, 1);
  Int32_t encoded_size = EncodeUint64Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeUint64Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedUint64Array(data_array, 3);
  encoded_size = EncodeUint64Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeUint64Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Float32_01) {
  Float32_t data = 9.336991F;
  Float32_t decoded_data = 0;
  Float32_t data_array[3] = { 8.236941F, -963.128911F, 8369.269872F };
  Float32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedFloat32Array(&data, 1);
  Int32_t encoded_size = EncodeFloat32Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeFloat32Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedFloat32Array(data_array, 3);
  encoded_size = EncodeFloat32Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeFloat32Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Float64_01) {
  Float64_t data = 9.336991000001;
  Float64_t decoded_data = 0;
  Float64_t data_array[3] = { 8.23694100001, -963.12891100001, 8369.26987200001 };
  Float64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedFloat64Array(&data, 1);
  Int32_t encoded_size = EncodeFloat64Array(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeFloat64Array(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedFloat64Array(data_array, 3);
  encoded_size = EncodeFloat64Array(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeFloat64Array(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}





TEST(SerializationUtils, Case_Template_Uint8_01) {
  Uint8_t data = 0x86;
  Uint8_t decoded_data = 0;
  Uint8_t data_array[3] = { 0x86, 0x56, 0x36 };
  Uint8_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 1);
  EXPECT_EQ(encoded_size, 1);
  EXPECT_EQ(decoded_size, 1);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 3);
  EXPECT_EQ(encoded_size, 3);
  EXPECT_EQ(decoded_size, 3);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Int8_01) {
  Int8_t data = 0x86;
  Int8_t decoded_data = 0;
  Int8_t data_array[3] = { -19, 0x56, 0x36 };
  Int8_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 1);
  EXPECT_EQ(encoded_size, 1);
  EXPECT_EQ(decoded_size, 1);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 3);
  EXPECT_EQ(encoded_size, 3);
  EXPECT_EQ(decoded_size, 3);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Int16_01) {
  Int16_t data = 3000;
  Int16_t decoded_data = 0;
  Int16_t data_array[3] = { -2999, 2000, 500 };
  Int16_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 2);
  EXPECT_EQ(encoded_size, 2);
  EXPECT_EQ(decoded_size, 2);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 6);
  EXPECT_EQ(encoded_size, 6);
  EXPECT_EQ(decoded_size, 6);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Uint16_01) {
  Uint16_t data = 6000;
  Uint16_t decoded_data = 0;
  Uint16_t data_array[3] = { 6001, 3000, 100 };
  Uint16_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 2);
  EXPECT_EQ(encoded_size, 2);
  EXPECT_EQ(decoded_size, 2);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 6);
  EXPECT_EQ(encoded_size, 6);
  EXPECT_EQ(decoded_size, 6);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Int32_01) {
  Int32_t data = 300000;
  Int32_t decoded_data = 0;
  Int32_t data_array[3] = { -2999000, 2000000, 500000 };
  Int32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << (Int32_t)data
            << ", decoded_data=" << (Int32_t)decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << (Int32_t)data_array[0]
            << ", " << (Int32_t)data_array[1]
            << ", " << (Int32_t)data_array[2]
            << "}, decoded_data_array={" << (Int32_t)decoded_data_array[0]
            << ", " << (Int32_t)decoded_data_array[1]
            << ", " << (Int32_t)decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Uint32_01) {
  Uint32_t data = 6000000;
  Uint32_t decoded_data = 0;
  Uint32_t data_array[3] = { 69990000, 30000001, 500002 };
  Uint32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Int64_01) {
  Int64_t data = -699990000008;
  Int64_t decoded_data = 0;
  Int64_t data_array[3] = { -6999228369, 36921548962321, -89210256441 };
  Int64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Uint64_01) {
  Uint64_t data = 7699990000008;
  Uint64_t decoded_data = 0;
  Uint64_t data_array[3] = { 86999228369, 936921548962321, 689210256441 };
  Uint64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Float32_01) {
  Float32_t data = 9.336991F;
  Float32_t decoded_data = 0;
  Float32_t data_array[3] = { 8.236941F, -963.128911F, 8369.269872F };
  Float32_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 4);
  EXPECT_EQ(encoded_size, 4);
  EXPECT_EQ(decoded_size, 4);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 12);
  EXPECT_EQ(encoded_size, 12);
  EXPECT_EQ(decoded_size, 12);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}

TEST(SerializationUtils, Case_Template_Float64_01) {
  Float64_t data = 9.336991000001;
  Float64_t decoded_data = 0;
  Float64_t data_array[3] = { 8.23694100001, -963.12891100001, 8369.26987200001 };
  Float64_t decoded_data_array[3] = { 0 };

  Uint8_t data_buffer[512] = { 0 };
  Int32_t data_size = GetSizeOfEncodedValueArray(&data, 1);
  Int32_t encoded_size = EncodeValueArray(data_buffer, 0, 512, &data, 1);
  Int32_t decoded_size = DecodeValueArray(data_buffer, 0, 512, &decoded_data, 1);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data=" << data
            << ", decoded_data=" << decoded_data
            << std::endl;

  EXPECT_EQ(data_size, 8);
  EXPECT_EQ(encoded_size, 8);
  EXPECT_EQ(decoded_size, 8);
  EXPECT_EQ(decoded_data, data);

  data_size = GetSizeOfEncodedValueArray(data_array, 3);
  encoded_size = EncodeValueArray(data_buffer, 0, 512, data_array, 3);
  decoded_size = DecodeValueArray(data_buffer, 0, 512, decoded_data_array, 3);

  std::cout << "data_size=" << data_size
            << ", encoded_size=" << encoded_size
            << ", decoded_size=" << decoded_size
            << ", data_array={" << data_array[0]
            << ", " << data_array[1]
            << ", " << data_array[2]
            << "}, decoded_data_array={" << decoded_data_array[0]
            << ", " << decoded_data_array[1]
            << ", " << decoded_data_array[2]
            << "}"
            << std::endl;

  EXPECT_EQ(data_size, 24);
  EXPECT_EQ(encoded_size, 24);
  EXPECT_EQ(decoded_size, 24);
  EXPECT_EQ(decoded_data_array[0], data_array[0]);
  EXPECT_EQ(decoded_data_array[1], data_array[1]);
  EXPECT_EQ(decoded_data_array[2], data_array[2]);
}


}  // namespace common
}  // namespace phoenix

#endif


