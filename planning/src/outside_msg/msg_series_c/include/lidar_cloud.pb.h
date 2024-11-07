// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: lidar_cloud.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_lidar_5fcloud_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_lidar_5fcloud_2eproto

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
#include "header.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_lidar_5fcloud_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_lidar_5fcloud_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxillaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const ::PROTOBUF_NAMESPACE_ID::uint32 offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_lidar_5fcloud_2eproto;
namespace phoenix {
namespace msg {
namespace perception {
class LidarCloud;
class LidarCloudDefaultTypeInternal;
extern LidarCloudDefaultTypeInternal _LidarCloud_default_instance_;
class LidarCloud_CloudPoint;
class LidarCloud_CloudPointDefaultTypeInternal;
extern LidarCloud_CloudPointDefaultTypeInternal _LidarCloud_CloudPoint_default_instance_;
}  // namespace perception
}  // namespace msg
}  // namespace phoenix
PROTOBUF_NAMESPACE_OPEN
template<> ::phoenix::msg::perception::LidarCloud* Arena::CreateMaybeMessage<::phoenix::msg::perception::LidarCloud>(Arena*);
template<> ::phoenix::msg::perception::LidarCloud_CloudPoint* Arena::CreateMaybeMessage<::phoenix::msg::perception::LidarCloud_CloudPoint>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace phoenix {
namespace msg {
namespace perception {

enum LidarCloud_LidarType : int {
  LidarCloud_LidarType_LIDAR_TYPE_UNKNOWN = 0,
  LidarCloud_LidarType_LIDAR_TYPE_IBEO_4 = 1,
  LidarCloud_LidarType_LIDAR_TYPE_VLP_16 = 2
};
bool LidarCloud_LidarType_IsValid(int value);
constexpr LidarCloud_LidarType LidarCloud_LidarType_LidarType_MIN = LidarCloud_LidarType_LIDAR_TYPE_UNKNOWN;
constexpr LidarCloud_LidarType LidarCloud_LidarType_LidarType_MAX = LidarCloud_LidarType_LIDAR_TYPE_VLP_16;
constexpr int LidarCloud_LidarType_LidarType_ARRAYSIZE = LidarCloud_LidarType_LidarType_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* LidarCloud_LidarType_descriptor();
template<typename T>
inline const std::string& LidarCloud_LidarType_Name(T enum_t_value) {
  static_assert(::std::is_same<T, LidarCloud_LidarType>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function LidarCloud_LidarType_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    LidarCloud_LidarType_descriptor(), enum_t_value);
}
inline bool LidarCloud_LidarType_Parse(
    const std::string& name, LidarCloud_LidarType* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<LidarCloud_LidarType>(
    LidarCloud_LidarType_descriptor(), name, value);
}
// ===================================================================

class LidarCloud_CloudPoint :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.perception.LidarCloud.CloudPoint) */ {
 public:
  LidarCloud_CloudPoint();
  virtual ~LidarCloud_CloudPoint();

  LidarCloud_CloudPoint(const LidarCloud_CloudPoint& from);
  LidarCloud_CloudPoint(LidarCloud_CloudPoint&& from) noexcept
    : LidarCloud_CloudPoint() {
    *this = ::std::move(from);
  }

  inline LidarCloud_CloudPoint& operator=(const LidarCloud_CloudPoint& from) {
    CopyFrom(from);
    return *this;
  }
  inline LidarCloud_CloudPoint& operator=(LidarCloud_CloudPoint&& from) noexcept {
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
  static const LidarCloud_CloudPoint& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LidarCloud_CloudPoint* internal_default_instance() {
    return reinterpret_cast<const LidarCloud_CloudPoint*>(
               &_LidarCloud_CloudPoint_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(LidarCloud_CloudPoint& a, LidarCloud_CloudPoint& b) {
    a.Swap(&b);
  }
  inline void Swap(LidarCloud_CloudPoint* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LidarCloud_CloudPoint* New() const final {
    return CreateMaybeMessage<LidarCloud_CloudPoint>(nullptr);
  }

  LidarCloud_CloudPoint* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LidarCloud_CloudPoint>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LidarCloud_CloudPoint& from);
  void MergeFrom(const LidarCloud_CloudPoint& from);
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
  void InternalSwap(LidarCloud_CloudPoint* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "phoenix.msg.perception.LidarCloud.CloudPoint";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lidar_5fcloud_2eproto);
    return ::descriptor_table_lidar_5fcloud_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kLayerFieldNumber = 1,
    kXFieldNumber = 2,
    kYFieldNumber = 3,
    kZFieldNumber = 4,
  };
  // optional int32 layer = 1 [default = 0];
  bool has_layer() const;
  void clear_layer();
  ::PROTOBUF_NAMESPACE_ID::int32 layer() const;
  void set_layer(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional float x = 2 [default = 0];
  bool has_x() const;
  void clear_x();
  float x() const;
  void set_x(float value);

  // optional float y = 3 [default = 0];
  bool has_y() const;
  void clear_y();
  float y() const;
  void set_y(float value);

  // optional float z = 4 [default = 0];
  bool has_z() const;
  void clear_z();
  float z() const;
  void set_z(float value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.perception.LidarCloud.CloudPoint)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::int32 layer_;
  float x_;
  float y_;
  float z_;
  friend struct ::TableStruct_lidar_5fcloud_2eproto;
};
// -------------------------------------------------------------------

class LidarCloud :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.perception.LidarCloud) */ {
 public:
  LidarCloud();
  virtual ~LidarCloud();

  LidarCloud(const LidarCloud& from);
  LidarCloud(LidarCloud&& from) noexcept
    : LidarCloud() {
    *this = ::std::move(from);
  }

  inline LidarCloud& operator=(const LidarCloud& from) {
    CopyFrom(from);
    return *this;
  }
  inline LidarCloud& operator=(LidarCloud&& from) noexcept {
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
  static const LidarCloud& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const LidarCloud* internal_default_instance() {
    return reinterpret_cast<const LidarCloud*>(
               &_LidarCloud_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(LidarCloud& a, LidarCloud& b) {
    a.Swap(&b);
  }
  inline void Swap(LidarCloud* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline LidarCloud* New() const final {
    return CreateMaybeMessage<LidarCloud>(nullptr);
  }

  LidarCloud* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<LidarCloud>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const LidarCloud& from);
  void MergeFrom(const LidarCloud& from);
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
  void InternalSwap(LidarCloud* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "phoenix.msg.perception.LidarCloud";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_lidar_5fcloud_2eproto);
    return ::descriptor_table_lidar_5fcloud_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  typedef LidarCloud_CloudPoint CloudPoint;

  typedef LidarCloud_LidarType LidarType;
  static constexpr LidarType LIDAR_TYPE_UNKNOWN =
    LidarCloud_LidarType_LIDAR_TYPE_UNKNOWN;
  static constexpr LidarType LIDAR_TYPE_IBEO_4 =
    LidarCloud_LidarType_LIDAR_TYPE_IBEO_4;
  static constexpr LidarType LIDAR_TYPE_VLP_16 =
    LidarCloud_LidarType_LIDAR_TYPE_VLP_16;
  static inline bool LidarType_IsValid(int value) {
    return LidarCloud_LidarType_IsValid(value);
  }
  static constexpr LidarType LidarType_MIN =
    LidarCloud_LidarType_LidarType_MIN;
  static constexpr LidarType LidarType_MAX =
    LidarCloud_LidarType_LidarType_MAX;
  static constexpr int LidarType_ARRAYSIZE =
    LidarCloud_LidarType_LidarType_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  LidarType_descriptor() {
    return LidarCloud_LidarType_descriptor();
  }
  template<typename T>
  static inline const std::string& LidarType_Name(T enum_t_value) {
    static_assert(::std::is_same<T, LidarType>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function LidarType_Name.");
    return LidarCloud_LidarType_Name(enum_t_value);
  }
  static inline bool LidarType_Parse(const std::string& name,
      LidarType* value) {
    return LidarCloud_LidarType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kPointsFieldNumber = 3,
    kHeaderFieldNumber = 1,
    kLidarTypeFieldNumber = 2,
  };
  // repeated .phoenix.msg.perception.LidarCloud.CloudPoint points = 3;
  int points_size() const;
  void clear_points();
  ::phoenix::msg::perception::LidarCloud_CloudPoint* mutable_points(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::phoenix::msg::perception::LidarCloud_CloudPoint >*
      mutable_points();
  const ::phoenix::msg::perception::LidarCloud_CloudPoint& points(int index) const;
  ::phoenix::msg::perception::LidarCloud_CloudPoint* add_points();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::phoenix::msg::perception::LidarCloud_CloudPoint >&
      points() const;

  // optional .phoenix.msg.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  const ::phoenix::msg::common::Header& header() const;
  ::phoenix::msg::common::Header* release_header();
  ::phoenix::msg::common::Header* mutable_header();
  void set_allocated_header(::phoenix::msg::common::Header* header);

  // optional .phoenix.msg.perception.LidarCloud.LidarType lidar_type = 2 [default = LIDAR_TYPE_UNKNOWN];
  bool has_lidar_type() const;
  void clear_lidar_type();
  ::phoenix::msg::perception::LidarCloud_LidarType lidar_type() const;
  void set_lidar_type(::phoenix::msg::perception::LidarCloud_LidarType value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.perception.LidarCloud)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::phoenix::msg::perception::LidarCloud_CloudPoint > points_;
  ::phoenix::msg::common::Header* header_;
  int lidar_type_;
  friend struct ::TableStruct_lidar_5fcloud_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// LidarCloud_CloudPoint

// optional int32 layer = 1 [default = 0];
inline bool LidarCloud_CloudPoint::has_layer() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void LidarCloud_CloudPoint::clear_layer() {
  layer_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 LidarCloud_CloudPoint::layer() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.CloudPoint.layer)
  return layer_;
}
inline void LidarCloud_CloudPoint::set_layer(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000001u;
  layer_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.perception.LidarCloud.CloudPoint.layer)
}

// optional float x = 2 [default = 0];
inline bool LidarCloud_CloudPoint::has_x() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LidarCloud_CloudPoint::clear_x() {
  x_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline float LidarCloud_CloudPoint::x() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.CloudPoint.x)
  return x_;
}
inline void LidarCloud_CloudPoint::set_x(float value) {
  _has_bits_[0] |= 0x00000002u;
  x_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.perception.LidarCloud.CloudPoint.x)
}

// optional float y = 3 [default = 0];
inline bool LidarCloud_CloudPoint::has_y() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void LidarCloud_CloudPoint::clear_y() {
  y_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline float LidarCloud_CloudPoint::y() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.CloudPoint.y)
  return y_;
}
inline void LidarCloud_CloudPoint::set_y(float value) {
  _has_bits_[0] |= 0x00000004u;
  y_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.perception.LidarCloud.CloudPoint.y)
}

// optional float z = 4 [default = 0];
inline bool LidarCloud_CloudPoint::has_z() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void LidarCloud_CloudPoint::clear_z() {
  z_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline float LidarCloud_CloudPoint::z() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.CloudPoint.z)
  return z_;
}
inline void LidarCloud_CloudPoint::set_z(float value) {
  _has_bits_[0] |= 0x00000008u;
  z_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.perception.LidarCloud.CloudPoint.z)
}

// -------------------------------------------------------------------

// LidarCloud

// optional .phoenix.msg.common.Header header = 1;
inline bool LidarCloud::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline const ::phoenix::msg::common::Header& LidarCloud::header() const {
  const ::phoenix::msg::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.header)
  return p != nullptr ? *p : *reinterpret_cast<const ::phoenix::msg::common::Header*>(
      &::phoenix::msg::common::_Header_default_instance_);
}
inline ::phoenix::msg::common::Header* LidarCloud::release_header() {
  // @@protoc_insertion_point(field_release:phoenix.msg.perception.LidarCloud.header)
  _has_bits_[0] &= ~0x00000001u;
  ::phoenix::msg::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::phoenix::msg::common::Header* LidarCloud::mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::phoenix::msg::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:phoenix.msg.perception.LidarCloud.header)
  return header_;
}
inline void LidarCloud::set_allocated_header(::phoenix::msg::common::Header* header) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(header_);
  }
  if (header) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      header = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, header, submessage_arena);
    }
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  header_ = header;
  // @@protoc_insertion_point(field_set_allocated:phoenix.msg.perception.LidarCloud.header)
}

// optional .phoenix.msg.perception.LidarCloud.LidarType lidar_type = 2 [default = LIDAR_TYPE_UNKNOWN];
inline bool LidarCloud::has_lidar_type() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void LidarCloud::clear_lidar_type() {
  lidar_type_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::phoenix::msg::perception::LidarCloud_LidarType LidarCloud::lidar_type() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.lidar_type)
  return static_cast< ::phoenix::msg::perception::LidarCloud_LidarType >(lidar_type_);
}
inline void LidarCloud::set_lidar_type(::phoenix::msg::perception::LidarCloud_LidarType value) {
  assert(::phoenix::msg::perception::LidarCloud_LidarType_IsValid(value));
  _has_bits_[0] |= 0x00000002u;
  lidar_type_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.perception.LidarCloud.lidar_type)
}

// repeated .phoenix.msg.perception.LidarCloud.CloudPoint points = 3;
inline int LidarCloud::points_size() const {
  return points_.size();
}
inline void LidarCloud::clear_points() {
  points_.Clear();
}
inline ::phoenix::msg::perception::LidarCloud_CloudPoint* LidarCloud::mutable_points(int index) {
  // @@protoc_insertion_point(field_mutable:phoenix.msg.perception.LidarCloud.points)
  return points_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::phoenix::msg::perception::LidarCloud_CloudPoint >*
LidarCloud::mutable_points() {
  // @@protoc_insertion_point(field_mutable_list:phoenix.msg.perception.LidarCloud.points)
  return &points_;
}
inline const ::phoenix::msg::perception::LidarCloud_CloudPoint& LidarCloud::points(int index) const {
  // @@protoc_insertion_point(field_get:phoenix.msg.perception.LidarCloud.points)
  return points_.Get(index);
}
inline ::phoenix::msg::perception::LidarCloud_CloudPoint* LidarCloud::add_points() {
  // @@protoc_insertion_point(field_add:phoenix.msg.perception.LidarCloud.points)
  return points_.Add();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::phoenix::msg::perception::LidarCloud_CloudPoint >&
LidarCloud::points() const {
  // @@protoc_insertion_point(field_list:phoenix.msg.perception.LidarCloud.points)
  return points_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace msg
}  // namespace phoenix

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::phoenix::msg::perception::LidarCloud_LidarType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::phoenix::msg::perception::LidarCloud_LidarType>() {
  return ::phoenix::msg::perception::LidarCloud_LidarType_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_lidar_5fcloud_2eproto
