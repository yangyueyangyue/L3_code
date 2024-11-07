// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: special_chassis_info.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_special_5fchassis_5finfo_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_special_5fchassis_5finfo_2eproto

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
#include <google/protobuf/unknown_field_set.h>
#include "header.pb.h"
#include "chassis_ft_auman.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_special_5fchassis_5finfo_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_special_5fchassis_5finfo_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_special_5fchassis_5finfo_2eproto;
namespace phoenix {
namespace msg {
namespace control {
class SpecialChassisInfo;
class SpecialChassisInfoDefaultTypeInternal;
extern SpecialChassisInfoDefaultTypeInternal _SpecialChassisInfo_default_instance_;
}  // namespace control
}  // namespace msg
}  // namespace phoenix
PROTOBUF_NAMESPACE_OPEN
template<> ::phoenix::msg::control::SpecialChassisInfo* Arena::CreateMaybeMessage<::phoenix::msg::control::SpecialChassisInfo>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace phoenix {
namespace msg {
namespace control {

// ===================================================================

class SpecialChassisInfo :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.control.SpecialChassisInfo) */ {
 public:
  SpecialChassisInfo();
  virtual ~SpecialChassisInfo();

  SpecialChassisInfo(const SpecialChassisInfo& from);
  SpecialChassisInfo(SpecialChassisInfo&& from) noexcept
    : SpecialChassisInfo() {
    *this = ::std::move(from);
  }

  inline SpecialChassisInfo& operator=(const SpecialChassisInfo& from) {
    CopyFrom(from);
    return *this;
  }
  inline SpecialChassisInfo& operator=(SpecialChassisInfo&& from) noexcept {
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
  static const SpecialChassisInfo& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SpecialChassisInfo* internal_default_instance() {
    return reinterpret_cast<const SpecialChassisInfo*>(
               &_SpecialChassisInfo_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SpecialChassisInfo& a, SpecialChassisInfo& b) {
    a.Swap(&b);
  }
  inline void Swap(SpecialChassisInfo* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SpecialChassisInfo* New() const final {
    return CreateMaybeMessage<SpecialChassisInfo>(nullptr);
  }

  SpecialChassisInfo* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SpecialChassisInfo>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SpecialChassisInfo& from);
  void MergeFrom(const SpecialChassisInfo& from);
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
  void InternalSwap(SpecialChassisInfo* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "phoenix.msg.control.SpecialChassisInfo";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_special_5fchassis_5finfo_2eproto);
    return ::descriptor_table_special_5fchassis_5finfo_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kHeaderFieldNumber = 1,
    kFtAumanFieldNumber = 16,
    kStartAdasFieldNumber = 2,
    kCntStuFrameLossCan0FieldNumber = 3,
    kCntStuFrameLossCan1FieldNumber = 4,
    kCntStuFrameLossCan2FieldNumber = 5,
    kCntStuFrameLossCan3FieldNumber = 6,
    kCntStuGtwToVehCan0FieldNumber = 7,
    kCntStuGtwToVehCan1FieldNumber = 8,
    kCntStuGtwToVehCan2FieldNumber = 9,
    kCntStuGtwToVehCan3FieldNumber = 10,
    kCntStuCtlToGtwCan0FieldNumber = 11,
    kCntStuCtlToGtwCan1FieldNumber = 12,
    kCntStuCtlToGtwCan2FieldNumber = 13,
    kCntStuCtlToGtwCan3FieldNumber = 14,
    kCntStuCtlToGtwFieldNumber = 15,
  };
  // optional .phoenix.msg.common.Header header = 1;
  bool has_header() const;
  void clear_header();
  const ::phoenix::msg::common::Header& header() const;
  ::phoenix::msg::common::Header* release_header();
  ::phoenix::msg::common::Header* mutable_header();
  void set_allocated_header(::phoenix::msg::common::Header* header);

  // optional .phoenix.msg.control.ChassisFtAuman ft_auman = 16;
  bool has_ft_auman() const;
  void clear_ft_auman();
  const ::phoenix::msg::control::ChassisFtAuman& ft_auman() const;
  ::phoenix::msg::control::ChassisFtAuman* release_ft_auman();
  ::phoenix::msg::control::ChassisFtAuman* mutable_ft_auman();
  void set_allocated_ft_auman(::phoenix::msg::control::ChassisFtAuman* ft_auman);

  // optional int32 start_adas = 2 [default = 0];
  bool has_start_adas() const;
  void clear_start_adas();
  ::PROTOBUF_NAMESPACE_ID::int32 start_adas() const;
  void set_start_adas(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_frame_loss_can0 = 3 [default = 0];
  bool has_cnt_stu_frame_loss_can0() const;
  void clear_cnt_stu_frame_loss_can0();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can0() const;
  void set_cnt_stu_frame_loss_can0(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_frame_loss_can1 = 4 [default = 0];
  bool has_cnt_stu_frame_loss_can1() const;
  void clear_cnt_stu_frame_loss_can1();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can1() const;
  void set_cnt_stu_frame_loss_can1(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_frame_loss_can2 = 5 [default = 0];
  bool has_cnt_stu_frame_loss_can2() const;
  void clear_cnt_stu_frame_loss_can2();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can2() const;
  void set_cnt_stu_frame_loss_can2(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_frame_loss_can3 = 6 [default = 0];
  bool has_cnt_stu_frame_loss_can3() const;
  void clear_cnt_stu_frame_loss_can3();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can3() const;
  void set_cnt_stu_frame_loss_can3(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_gtw_to_veh_can0 = 7 [default = 0];
  bool has_cnt_stu_gtw_to_veh_can0() const;
  void clear_cnt_stu_gtw_to_veh_can0();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can0() const;
  void set_cnt_stu_gtw_to_veh_can0(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_gtw_to_veh_can1 = 8 [default = 0];
  bool has_cnt_stu_gtw_to_veh_can1() const;
  void clear_cnt_stu_gtw_to_veh_can1();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can1() const;
  void set_cnt_stu_gtw_to_veh_can1(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_gtw_to_veh_can2 = 9 [default = 0];
  bool has_cnt_stu_gtw_to_veh_can2() const;
  void clear_cnt_stu_gtw_to_veh_can2();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can2() const;
  void set_cnt_stu_gtw_to_veh_can2(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_gtw_to_veh_can3 = 10 [default = 0];
  bool has_cnt_stu_gtw_to_veh_can3() const;
  void clear_cnt_stu_gtw_to_veh_can3();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can3() const;
  void set_cnt_stu_gtw_to_veh_can3(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_ctl_to_gtw_can0 = 11 [default = 0];
  bool has_cnt_stu_ctl_to_gtw_can0() const;
  void clear_cnt_stu_ctl_to_gtw_can0();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can0() const;
  void set_cnt_stu_ctl_to_gtw_can0(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_ctl_to_gtw_can1 = 12 [default = 0];
  bool has_cnt_stu_ctl_to_gtw_can1() const;
  void clear_cnt_stu_ctl_to_gtw_can1();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can1() const;
  void set_cnt_stu_ctl_to_gtw_can1(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_ctl_to_gtw_can2 = 13 [default = 0];
  bool has_cnt_stu_ctl_to_gtw_can2() const;
  void clear_cnt_stu_ctl_to_gtw_can2();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can2() const;
  void set_cnt_stu_ctl_to_gtw_can2(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_ctl_to_gtw_can3 = 14 [default = 0];
  bool has_cnt_stu_ctl_to_gtw_can3() const;
  void clear_cnt_stu_ctl_to_gtw_can3();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can3() const;
  void set_cnt_stu_ctl_to_gtw_can3(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 cnt_stu_ctl_to_gtw = 15 [default = 0];
  bool has_cnt_stu_ctl_to_gtw() const;
  void clear_cnt_stu_ctl_to_gtw();
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw() const;
  void set_cnt_stu_ctl_to_gtw(::PROTOBUF_NAMESPACE_ID::int32 value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.control.SpecialChassisInfo)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::phoenix::msg::common::Header* header_;
  ::phoenix::msg::control::ChassisFtAuman* ft_auman_;
  ::PROTOBUF_NAMESPACE_ID::int32 start_adas_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can0_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can1_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can2_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_frame_loss_can3_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can0_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can1_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can2_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_gtw_to_veh_can3_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can0_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can1_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can2_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_can3_;
  ::PROTOBUF_NAMESPACE_ID::int32 cnt_stu_ctl_to_gtw_;
  friend struct ::TableStruct_special_5fchassis_5finfo_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SpecialChassisInfo

// optional .phoenix.msg.common.Header header = 1;
inline bool SpecialChassisInfo::has_header() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline const ::phoenix::msg::common::Header& SpecialChassisInfo::header() const {
  const ::phoenix::msg::common::Header* p = header_;
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.header)
  return p != nullptr ? *p : *reinterpret_cast<const ::phoenix::msg::common::Header*>(
      &::phoenix::msg::common::_Header_default_instance_);
}
inline ::phoenix::msg::common::Header* SpecialChassisInfo::release_header() {
  // @@protoc_insertion_point(field_release:phoenix.msg.control.SpecialChassisInfo.header)
  _has_bits_[0] &= ~0x00000001u;
  ::phoenix::msg::common::Header* temp = header_;
  header_ = nullptr;
  return temp;
}
inline ::phoenix::msg::common::Header* SpecialChassisInfo::mutable_header() {
  _has_bits_[0] |= 0x00000001u;
  if (header_ == nullptr) {
    auto* p = CreateMaybeMessage<::phoenix::msg::common::Header>(GetArenaNoVirtual());
    header_ = p;
  }
  // @@protoc_insertion_point(field_mutable:phoenix.msg.control.SpecialChassisInfo.header)
  return header_;
}
inline void SpecialChassisInfo::set_allocated_header(::phoenix::msg::common::Header* header) {
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
  // @@protoc_insertion_point(field_set_allocated:phoenix.msg.control.SpecialChassisInfo.header)
}

// optional int32 start_adas = 2 [default = 0];
inline bool SpecialChassisInfo::has_start_adas() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SpecialChassisInfo::clear_start_adas() {
  start_adas_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::start_adas() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.start_adas)
  return start_adas_;
}
inline void SpecialChassisInfo::set_start_adas(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  start_adas_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.start_adas)
}

// optional int32 cnt_stu_frame_loss_can0 = 3 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_frame_loss_can0() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_frame_loss_can0() {
  cnt_stu_frame_loss_can0_ = 0;
  _has_bits_[0] &= ~0x00000008u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_frame_loss_can0() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can0)
  return cnt_stu_frame_loss_can0_;
}
inline void SpecialChassisInfo::set_cnt_stu_frame_loss_can0(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000008u;
  cnt_stu_frame_loss_can0_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can0)
}

// optional int32 cnt_stu_frame_loss_can1 = 4 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_frame_loss_can1() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_frame_loss_can1() {
  cnt_stu_frame_loss_can1_ = 0;
  _has_bits_[0] &= ~0x00000010u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_frame_loss_can1() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can1)
  return cnt_stu_frame_loss_can1_;
}
inline void SpecialChassisInfo::set_cnt_stu_frame_loss_can1(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000010u;
  cnt_stu_frame_loss_can1_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can1)
}

// optional int32 cnt_stu_frame_loss_can2 = 5 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_frame_loss_can2() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_frame_loss_can2() {
  cnt_stu_frame_loss_can2_ = 0;
  _has_bits_[0] &= ~0x00000020u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_frame_loss_can2() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can2)
  return cnt_stu_frame_loss_can2_;
}
inline void SpecialChassisInfo::set_cnt_stu_frame_loss_can2(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000020u;
  cnt_stu_frame_loss_can2_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can2)
}

// optional int32 cnt_stu_frame_loss_can3 = 6 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_frame_loss_can3() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_frame_loss_can3() {
  cnt_stu_frame_loss_can3_ = 0;
  _has_bits_[0] &= ~0x00000040u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_frame_loss_can3() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can3)
  return cnt_stu_frame_loss_can3_;
}
inline void SpecialChassisInfo::set_cnt_stu_frame_loss_can3(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000040u;
  cnt_stu_frame_loss_can3_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_frame_loss_can3)
}

// optional int32 cnt_stu_gtw_to_veh_can0 = 7 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_gtw_to_veh_can0() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_gtw_to_veh_can0() {
  cnt_stu_gtw_to_veh_can0_ = 0;
  _has_bits_[0] &= ~0x00000080u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_gtw_to_veh_can0() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can0)
  return cnt_stu_gtw_to_veh_can0_;
}
inline void SpecialChassisInfo::set_cnt_stu_gtw_to_veh_can0(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000080u;
  cnt_stu_gtw_to_veh_can0_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can0)
}

// optional int32 cnt_stu_gtw_to_veh_can1 = 8 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_gtw_to_veh_can1() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_gtw_to_veh_can1() {
  cnt_stu_gtw_to_veh_can1_ = 0;
  _has_bits_[0] &= ~0x00000100u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_gtw_to_veh_can1() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can1)
  return cnt_stu_gtw_to_veh_can1_;
}
inline void SpecialChassisInfo::set_cnt_stu_gtw_to_veh_can1(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000100u;
  cnt_stu_gtw_to_veh_can1_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can1)
}

// optional int32 cnt_stu_gtw_to_veh_can2 = 9 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_gtw_to_veh_can2() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_gtw_to_veh_can2() {
  cnt_stu_gtw_to_veh_can2_ = 0;
  _has_bits_[0] &= ~0x00000200u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_gtw_to_veh_can2() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can2)
  return cnt_stu_gtw_to_veh_can2_;
}
inline void SpecialChassisInfo::set_cnt_stu_gtw_to_veh_can2(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000200u;
  cnt_stu_gtw_to_veh_can2_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can2)
}

// optional int32 cnt_stu_gtw_to_veh_can3 = 10 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_gtw_to_veh_can3() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_gtw_to_veh_can3() {
  cnt_stu_gtw_to_veh_can3_ = 0;
  _has_bits_[0] &= ~0x00000400u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_gtw_to_veh_can3() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can3)
  return cnt_stu_gtw_to_veh_can3_;
}
inline void SpecialChassisInfo::set_cnt_stu_gtw_to_veh_can3(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000400u;
  cnt_stu_gtw_to_veh_can3_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_gtw_to_veh_can3)
}

// optional int32 cnt_stu_ctl_to_gtw_can0 = 11 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_ctl_to_gtw_can0() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_ctl_to_gtw_can0() {
  cnt_stu_ctl_to_gtw_can0_ = 0;
  _has_bits_[0] &= ~0x00000800u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_ctl_to_gtw_can0() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can0)
  return cnt_stu_ctl_to_gtw_can0_;
}
inline void SpecialChassisInfo::set_cnt_stu_ctl_to_gtw_can0(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000800u;
  cnt_stu_ctl_to_gtw_can0_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can0)
}

// optional int32 cnt_stu_ctl_to_gtw_can1 = 12 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_ctl_to_gtw_can1() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_ctl_to_gtw_can1() {
  cnt_stu_ctl_to_gtw_can1_ = 0;
  _has_bits_[0] &= ~0x00001000u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_ctl_to_gtw_can1() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can1)
  return cnt_stu_ctl_to_gtw_can1_;
}
inline void SpecialChassisInfo::set_cnt_stu_ctl_to_gtw_can1(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00001000u;
  cnt_stu_ctl_to_gtw_can1_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can1)
}

// optional int32 cnt_stu_ctl_to_gtw_can2 = 13 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_ctl_to_gtw_can2() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_ctl_to_gtw_can2() {
  cnt_stu_ctl_to_gtw_can2_ = 0;
  _has_bits_[0] &= ~0x00002000u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_ctl_to_gtw_can2() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can2)
  return cnt_stu_ctl_to_gtw_can2_;
}
inline void SpecialChassisInfo::set_cnt_stu_ctl_to_gtw_can2(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00002000u;
  cnt_stu_ctl_to_gtw_can2_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can2)
}

// optional int32 cnt_stu_ctl_to_gtw_can3 = 14 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_ctl_to_gtw_can3() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_ctl_to_gtw_can3() {
  cnt_stu_ctl_to_gtw_can3_ = 0;
  _has_bits_[0] &= ~0x00004000u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_ctl_to_gtw_can3() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can3)
  return cnt_stu_ctl_to_gtw_can3_;
}
inline void SpecialChassisInfo::set_cnt_stu_ctl_to_gtw_can3(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00004000u;
  cnt_stu_ctl_to_gtw_can3_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw_can3)
}

// optional int32 cnt_stu_ctl_to_gtw = 15 [default = 0];
inline bool SpecialChassisInfo::has_cnt_stu_ctl_to_gtw() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void SpecialChassisInfo::clear_cnt_stu_ctl_to_gtw() {
  cnt_stu_ctl_to_gtw_ = 0;
  _has_bits_[0] &= ~0x00008000u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 SpecialChassisInfo::cnt_stu_ctl_to_gtw() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw)
  return cnt_stu_ctl_to_gtw_;
}
inline void SpecialChassisInfo::set_cnt_stu_ctl_to_gtw(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00008000u;
  cnt_stu_ctl_to_gtw_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.SpecialChassisInfo.cnt_stu_ctl_to_gtw)
}

// optional .phoenix.msg.control.ChassisFtAuman ft_auman = 16;
inline bool SpecialChassisInfo::has_ft_auman() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline const ::phoenix::msg::control::ChassisFtAuman& SpecialChassisInfo::ft_auman() const {
  const ::phoenix::msg::control::ChassisFtAuman* p = ft_auman_;
  // @@protoc_insertion_point(field_get:phoenix.msg.control.SpecialChassisInfo.ft_auman)
  return p != nullptr ? *p : *reinterpret_cast<const ::phoenix::msg::control::ChassisFtAuman*>(
      &::phoenix::msg::control::_ChassisFtAuman_default_instance_);
}
inline ::phoenix::msg::control::ChassisFtAuman* SpecialChassisInfo::release_ft_auman() {
  // @@protoc_insertion_point(field_release:phoenix.msg.control.SpecialChassisInfo.ft_auman)
  _has_bits_[0] &= ~0x00000002u;
  ::phoenix::msg::control::ChassisFtAuman* temp = ft_auman_;
  ft_auman_ = nullptr;
  return temp;
}
inline ::phoenix::msg::control::ChassisFtAuman* SpecialChassisInfo::mutable_ft_auman() {
  _has_bits_[0] |= 0x00000002u;
  if (ft_auman_ == nullptr) {
    auto* p = CreateMaybeMessage<::phoenix::msg::control::ChassisFtAuman>(GetArenaNoVirtual());
    ft_auman_ = p;
  }
  // @@protoc_insertion_point(field_mutable:phoenix.msg.control.SpecialChassisInfo.ft_auman)
  return ft_auman_;
}
inline void SpecialChassisInfo::set_allocated_ft_auman(::phoenix::msg::control::ChassisFtAuman* ft_auman) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(ft_auman_);
  }
  if (ft_auman) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      ft_auman = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, ft_auman, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  ft_auman_ = ft_auman;
  // @@protoc_insertion_point(field_set_allocated:phoenix.msg.control.SpecialChassisInfo.ft_auman)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace control
}  // namespace msg
}  // namespace phoenix

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_special_5fchassis_5finfo_2eproto
