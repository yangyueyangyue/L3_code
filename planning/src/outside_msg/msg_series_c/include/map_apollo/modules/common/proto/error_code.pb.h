// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/error_code.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2ferror_5fcode_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2ferror_5fcode_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3009000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3009001 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/inlined_string_field.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2fproto_2ferror_5fcode_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[1]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto;
namespace apollo {
namespace common {
class StatusPb;
class StatusPbDefaultTypeInternal;
extern StatusPbDefaultTypeInternal _StatusPb_default_instance_;
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::common::StatusPb* Arena::CreateMaybeMessage<::apollo::common::StatusPb>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace common {

enum ErrorCode : int {
  OK = 0,
  CONTROL_ERROR = 1000,
  CONTROL_INIT_ERROR = 1001,
  CONTROL_COMPUTE_ERROR = 1002,
  CONTROL_ESTOP_ERROR = 1003,
  PERFECT_CONTROL_ERROR = 1004,
  CANBUS_ERROR = 2000,
  CAN_CLIENT_ERROR_BASE = 2100,
  CAN_CLIENT_ERROR_OPEN_DEVICE_FAILED = 2101,
  CAN_CLIENT_ERROR_FRAME_NUM = 2102,
  CAN_CLIENT_ERROR_SEND_FAILED = 2103,
  CAN_CLIENT_ERROR_RECV_FAILED = 2104,
  LOCALIZATION_ERROR = 3000,
  LOCALIZATION_ERROR_MSG = 3100,
  LOCALIZATION_ERROR_LIDAR = 3200,
  LOCALIZATION_ERROR_INTEG = 3300,
  LOCALIZATION_ERROR_GNSS = 3400,
  PERCEPTION_ERROR = 4000,
  PERCEPTION_ERROR_TF = 4001,
  PERCEPTION_ERROR_PROCESS = 4002,
  PERCEPTION_FATAL = 4003,
  PERCEPTION_ERROR_NONE = 4004,
  PERCEPTION_ERROR_UNKNOWN = 4005,
  PREDICTION_ERROR = 5000,
  PLANNING_ERROR = 6000,
  PLANNING_ERROR_NOT_READY = 6001,
  HDMAP_DATA_ERROR = 7000,
  ROUTING_ERROR = 8000,
  ROUTING_ERROR_REQUEST = 8001,
  ROUTING_ERROR_RESPONSE = 8002,
  ROUTING_ERROR_NOT_READY = 8003,
  END_OF_INPUT = 9000,
  HTTP_LOGIC_ERROR = 10000,
  HTTP_RUNTIME_ERROR = 10001,
  RELATIVE_MAP_ERROR = 11000,
  RELATIVE_MAP_NOT_READY = 11001,
  DRIVER_ERROR_GNSS = 12000,
  DRIVER_ERROR_VELODYNE = 13000,
  STORYTELLING_ERROR = 14000
};
bool ErrorCode_IsValid(int value);
constexpr ErrorCode ErrorCode_MIN = OK;
constexpr ErrorCode ErrorCode_MAX = STORYTELLING_ERROR;
constexpr int ErrorCode_ARRAYSIZE = ErrorCode_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ErrorCode_descriptor();
template<typename T>
inline const std::string& ErrorCode_Name(T enum_t_value) {
  static_assert(::std::is_same<T, ErrorCode>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function ErrorCode_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    ErrorCode_descriptor(), enum_t_value);
}
inline bool ErrorCode_Parse(
    const std::string& name, ErrorCode* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<ErrorCode>(
    ErrorCode_descriptor(), name, value);
}
// ===================================================================

class StatusPb :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.common.StatusPb) */ {
 public:
  StatusPb();
  virtual ~StatusPb();

  StatusPb(const StatusPb& from);
  StatusPb(StatusPb&& from) noexcept
    : StatusPb() {
    *this = ::std::move(from);
  }

  inline StatusPb& operator=(const StatusPb& from) {
    CopyFrom(from);
    return *this;
  }
  inline StatusPb& operator=(StatusPb&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  inline const ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::PROTOBUF_NAMESPACE_ID::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return GetMetadataStatic().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return GetMetadataStatic().reflection;
  }
  static const StatusPb& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const StatusPb* internal_default_instance() {
    return reinterpret_cast<const StatusPb*>(
               &_StatusPb_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(StatusPb& a, StatusPb& b) {
    a.Swap(&b);
  }
  inline void Swap(StatusPb* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline StatusPb* New() const final {
    return CreateMaybeMessage<StatusPb>(nullptr);
  }

  StatusPb* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<StatusPb>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const StatusPb& from);
  void MergeFrom(const StatusPb& from);
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  #if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  #else
  bool MergePartialFromCodedStream(
      ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) final;
  #endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
  void SerializeWithCachedSizes(
      ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const final;
  ::PROTOBUF_NAMESPACE_ID::uint8* InternalSerializeWithCachedSizesToArray(
      ::PROTOBUF_NAMESPACE_ID::uint8* target) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  inline void SharedCtor();
  inline void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(StatusPb* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.common.StatusPb";
  }
  private:
  inline ::PROTOBUF_NAMESPACE_ID::Arena* GetArenaNoVirtual() const {
    return nullptr;
  }
  inline void* MaybeArenaPtr() const {
    return nullptr;
  }
  public:

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;
  private:
  static ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadataStatic() {
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto);
    return ::descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kMsgFieldNumber = 2,
    kErrorCodeFieldNumber = 1,
  };
  // optional string msg = 2;
  bool has_msg() const;
  void clear_msg();
  const std::string& msg() const;
  void set_msg(const std::string& value);
  void set_msg(std::string&& value);
  void set_msg(const char* value);
  void set_msg(const char* value, size_t size);
  std::string* mutable_msg();
  std::string* release_msg();
  void set_allocated_msg(std::string* msg);

  // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
  bool has_error_code() const;
  void clear_error_code();
  ::apollo::common::ErrorCode error_code() const;
  void set_error_code(::apollo::common::ErrorCode value);

  // @@protoc_insertion_point(class_scope:apollo.common.StatusPb)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr msg_;
  int error_code_;
  friend struct ::TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// StatusPb

// optional .apollo.common.ErrorCode error_code = 1 [default = OK];
inline bool StatusPb::has_error_code() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void StatusPb::clear_error_code() {
  error_code_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::apollo::common::ErrorCode StatusPb::error_code() const {
  // @@protoc_insertion_point(field_get:apollo.common.StatusPb.error_code)
  return static_cast< ::apollo::common::ErrorCode >(error_code_);
}
inline void StatusPb::set_error_code(::apollo::common::ErrorCode value) {
  assert(::apollo::common::ErrorCode_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  error_code_ = value;
  // @@protoc_insertion_point(field_set:apollo.common.StatusPb.error_code)
}

// optional string msg = 2;
inline bool StatusPb::has_msg() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void StatusPb::clear_msg() {
  msg_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& StatusPb::msg() const {
  // @@protoc_insertion_point(field_get:apollo.common.StatusPb.msg)
  return msg_.GetNoArena();
}
inline void StatusPb::set_msg(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  msg_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.common.StatusPb.msg)
}
inline void StatusPb::set_msg(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  msg_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.common.StatusPb.msg)
}
inline void StatusPb::set_msg(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  msg_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.common.StatusPb.msg)
}
inline void StatusPb::set_msg(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  msg_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.common.StatusPb.msg)
}
inline std::string* StatusPb::mutable_msg() {
  _has_bits_[0] |= 0x00000001u;
  // @@protoc_insertion_point(field_mutable:apollo.common.StatusPb.msg)
  return msg_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* StatusPb::release_msg() {
  // @@protoc_insertion_point(field_release:apollo.common.StatusPb.msg)
  if (!has_msg()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return msg_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void StatusPb::set_allocated_msg(std::string* msg) {
  if (msg != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  msg_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), msg);
  // @@protoc_insertion_point(field_set_allocated:apollo.common.StatusPb.msg)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace common
}  // namespace apollo

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::apollo::common::ErrorCode> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::apollo::common::ErrorCode>() {
  return ::apollo::common::ErrorCode_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fcommon_2fproto_2ferror_5fcode_2eproto
