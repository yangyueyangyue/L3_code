// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: chassis_ft_auman.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_chassis_5fft_5fauman_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_chassis_5fft_5fauman_2eproto

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
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_chassis_5fft_5fauman_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_chassis_5fft_5fauman_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_chassis_5fft_5fauman_2eproto;
namespace phoenix {
namespace msg {
namespace control {
class ChassisFtAuman;
class ChassisFtAumanDefaultTypeInternal;
extern ChassisFtAumanDefaultTypeInternal _ChassisFtAuman_default_instance_;
}  // namespace control
}  // namespace msg
}  // namespace phoenix
PROTOBUF_NAMESPACE_OPEN
template<> ::phoenix::msg::control::ChassisFtAuman* Arena::CreateMaybeMessage<::phoenix::msg::control::ChassisFtAuman>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace phoenix {
namespace msg {
namespace control {

// ===================================================================

class ChassisFtAuman :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:phoenix.msg.control.ChassisFtAuman) */ {
 public:
  ChassisFtAuman();
  virtual ~ChassisFtAuman();

  ChassisFtAuman(const ChassisFtAuman& from);
  ChassisFtAuman(ChassisFtAuman&& from) noexcept
    : ChassisFtAuman() {
    *this = ::std::move(from);
  }

  inline ChassisFtAuman& operator=(const ChassisFtAuman& from) {
    CopyFrom(from);
    return *this;
  }
  inline ChassisFtAuman& operator=(ChassisFtAuman&& from) noexcept {
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
  static const ChassisFtAuman& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const ChassisFtAuman* internal_default_instance() {
    return reinterpret_cast<const ChassisFtAuman*>(
               &_ChassisFtAuman_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(ChassisFtAuman& a, ChassisFtAuman& b) {
    a.Swap(&b);
  }
  inline void Swap(ChassisFtAuman* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline ChassisFtAuman* New() const final {
    return CreateMaybeMessage<ChassisFtAuman>(nullptr);
  }

  ChassisFtAuman* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<ChassisFtAuman>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const ChassisFtAuman& from);
  void MergeFrom(const ChassisFtAuman& from);
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
  void InternalSwap(ChassisFtAuman* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "phoenix.msg.control.ChassisFtAuman";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_chassis_5fft_5fauman_2eproto);
    return ::descriptor_table_chassis_5fft_5fauman_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSwitchTjaFieldNumber = 1,
    kSwitchHwaFieldNumber = 2,
    kSwitchIDriveFieldNumber = 3,
  };
  // optional int32 switch_tja = 1 [default = 0];
  bool has_switch_tja() const;
  void clear_switch_tja();
  ::PROTOBUF_NAMESPACE_ID::int32 switch_tja() const;
  void set_switch_tja(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 switch_hwa = 2 [default = 0];
  bool has_switch_hwa() const;
  void clear_switch_hwa();
  ::PROTOBUF_NAMESPACE_ID::int32 switch_hwa() const;
  void set_switch_hwa(::PROTOBUF_NAMESPACE_ID::int32 value);

  // optional int32 switch_i_drive = 3 [default = 0];
  bool has_switch_i_drive() const;
  void clear_switch_i_drive();
  ::PROTOBUF_NAMESPACE_ID::int32 switch_i_drive() const;
  void set_switch_i_drive(::PROTOBUF_NAMESPACE_ID::int32 value);

  // @@protoc_insertion_point(class_scope:phoenix.msg.control.ChassisFtAuman)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::int32 switch_tja_;
  ::PROTOBUF_NAMESPACE_ID::int32 switch_hwa_;
  ::PROTOBUF_NAMESPACE_ID::int32 switch_i_drive_;
  friend struct ::TableStruct_chassis_5fft_5fauman_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// ChassisFtAuman

// optional int32 switch_tja = 1 [default = 0];
inline bool ChassisFtAuman::has_switch_tja() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void ChassisFtAuman::clear_switch_tja() {
  switch_tja_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 ChassisFtAuman::switch_tja() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_tja)
  return switch_tja_;
}
inline void ChassisFtAuman::set_switch_tja(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000001u;
  switch_tja_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_tja)
}

// optional int32 switch_hwa = 2 [default = 0];
inline bool ChassisFtAuman::has_switch_hwa() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void ChassisFtAuman::clear_switch_hwa() {
  switch_hwa_ = 0;
  _has_bits_[0] &= ~0x00000002u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 ChassisFtAuman::switch_hwa() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_hwa)
  return switch_hwa_;
}
inline void ChassisFtAuman::set_switch_hwa(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000002u;
  switch_hwa_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_hwa)
}

// optional int32 switch_i_drive = 3 [default = 0];
inline bool ChassisFtAuman::has_switch_i_drive() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void ChassisFtAuman::clear_switch_i_drive() {
  switch_i_drive_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline ::PROTOBUF_NAMESPACE_ID::int32 ChassisFtAuman::switch_i_drive() const {
  // @@protoc_insertion_point(field_get:phoenix.msg.control.ChassisFtAuman.switch_i_drive)
  return switch_i_drive_;
}
inline void ChassisFtAuman::set_switch_i_drive(::PROTOBUF_NAMESPACE_ID::int32 value) {
  _has_bits_[0] |= 0x00000004u;
  switch_i_drive_ = value;
  // @@protoc_insertion_point(field_set:phoenix.msg.control.ChassisFtAuman.switch_i_drive)
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
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_chassis_5fft_5fauman_2eproto
