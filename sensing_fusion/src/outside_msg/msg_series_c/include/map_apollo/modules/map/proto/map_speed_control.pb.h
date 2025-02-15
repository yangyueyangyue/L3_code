// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_speed_control.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto

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
#include "modules/map/proto/map_geometry.pb.h"
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto;
namespace apollo {
namespace hdmap {
class SpeedControl;
class SpeedControlDefaultTypeInternal;
extern SpeedControlDefaultTypeInternal _SpeedControl_default_instance_;
class SpeedControls;
class SpeedControlsDefaultTypeInternal;
extern SpeedControlsDefaultTypeInternal _SpeedControls_default_instance_;
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::hdmap::SpeedControl* Arena::CreateMaybeMessage<::apollo::hdmap::SpeedControl>(Arena*);
template<> ::apollo::hdmap::SpeedControls* Arena::CreateMaybeMessage<::apollo::hdmap::SpeedControls>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace hdmap {

// ===================================================================

class SpeedControl :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.SpeedControl) */ {
 public:
  SpeedControl();
  virtual ~SpeedControl();

  SpeedControl(const SpeedControl& from);
  SpeedControl(SpeedControl&& from) noexcept
    : SpeedControl() {
    *this = ::std::move(from);
  }

  inline SpeedControl& operator=(const SpeedControl& from) {
    CopyFrom(from);
    return *this;
  }
  inline SpeedControl& operator=(SpeedControl&& from) noexcept {
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
  static const SpeedControl& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SpeedControl* internal_default_instance() {
    return reinterpret_cast<const SpeedControl*>(
               &_SpeedControl_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(SpeedControl& a, SpeedControl& b) {
    a.Swap(&b);
  }
  inline void Swap(SpeedControl* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SpeedControl* New() const final {
    return CreateMaybeMessage<SpeedControl>(nullptr);
  }

  SpeedControl* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SpeedControl>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SpeedControl& from);
  void MergeFrom(const SpeedControl& from);
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
  void InternalSwap(SpeedControl* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.hdmap.SpeedControl";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto);
    return ::descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNameFieldNumber = 1,
    kPolygonFieldNumber = 2,
    kSpeedLimitFieldNumber = 3,
  };
  // optional string name = 1;
  bool has_name() const;
  void clear_name();
  const std::string& name() const;
  void set_name(const std::string& value);
  void set_name(std::string&& value);
  void set_name(const char* value);
  void set_name(const char* value, size_t size);
  std::string* mutable_name();
  std::string* release_name();
  void set_allocated_name(std::string* name);

  // optional .apollo.hdmap.Polygon polygon = 2;
  bool has_polygon() const;
  void clear_polygon();
  const ::apollo::hdmap::Polygon& polygon() const;
  ::apollo::hdmap::Polygon* release_polygon();
  ::apollo::hdmap::Polygon* mutable_polygon();
  void set_allocated_polygon(::apollo::hdmap::Polygon* polygon);

  // optional double speed_limit = 3;
  bool has_speed_limit() const;
  void clear_speed_limit();
  double speed_limit() const;
  void set_speed_limit(double value);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.SpeedControl)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  ::apollo::hdmap::Polygon* polygon_;
  double speed_limit_;
  friend struct ::TableStruct_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto;
};
// -------------------------------------------------------------------

class SpeedControls :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.SpeedControls) */ {
 public:
  SpeedControls();
  virtual ~SpeedControls();

  SpeedControls(const SpeedControls& from);
  SpeedControls(SpeedControls&& from) noexcept
    : SpeedControls() {
    *this = ::std::move(from);
  }

  inline SpeedControls& operator=(const SpeedControls& from) {
    CopyFrom(from);
    return *this;
  }
  inline SpeedControls& operator=(SpeedControls&& from) noexcept {
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
  static const SpeedControls& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const SpeedControls* internal_default_instance() {
    return reinterpret_cast<const SpeedControls*>(
               &_SpeedControls_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(SpeedControls& a, SpeedControls& b) {
    a.Swap(&b);
  }
  inline void Swap(SpeedControls* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline SpeedControls* New() const final {
    return CreateMaybeMessage<SpeedControls>(nullptr);
  }

  SpeedControls* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<SpeedControls>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const SpeedControls& from);
  void MergeFrom(const SpeedControls& from);
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
  void InternalSwap(SpeedControls* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.hdmap.SpeedControls";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto);
    return ::descriptor_table_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kSpeedControlFieldNumber = 1,
  };
  // repeated .apollo.hdmap.SpeedControl speed_control = 1;
  int speed_control_size() const;
  void clear_speed_control();
  ::apollo::hdmap::SpeedControl* mutable_speed_control(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::SpeedControl >*
      mutable_speed_control();
  const ::apollo::hdmap::SpeedControl& speed_control(int index) const;
  ::apollo::hdmap::SpeedControl* add_speed_control();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::SpeedControl >&
      speed_control() const;

  // @@protoc_insertion_point(class_scope:apollo.hdmap.SpeedControls)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::SpeedControl > speed_control_;
  friend struct ::TableStruct_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// SpeedControl

// optional string name = 1;
inline bool SpeedControl::has_name() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void SpeedControl::clear_name() {
  name_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& SpeedControl::name() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.SpeedControl.name)
  return name_.GetNoArena();
}
inline void SpeedControl::set_name(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.hdmap.SpeedControl.name)
}
inline void SpeedControl::set_name(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  name_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.hdmap.SpeedControl.name)
}
inline void SpeedControl::set_name(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.hdmap.SpeedControl.name)
}
inline void SpeedControl::set_name(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  name_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.hdmap.SpeedControl.name)
}
inline std::string* SpeedControl::mutable_name() {
  _has_bits_[0] |= 0x00000001u;
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.SpeedControl.name)
  return name_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* SpeedControl::release_name() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.SpeedControl.name)
  if (!has_name()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return name_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void SpeedControl::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  name_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), name);
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.SpeedControl.name)
}

// optional .apollo.hdmap.Polygon polygon = 2;
inline bool SpeedControl::has_polygon() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline const ::apollo::hdmap::Polygon& SpeedControl::polygon() const {
  const ::apollo::hdmap::Polygon* p = polygon_;
  // @@protoc_insertion_point(field_get:apollo.hdmap.SpeedControl.polygon)
  return p != nullptr ? *p : *reinterpret_cast<const ::apollo::hdmap::Polygon*>(
      &::apollo::hdmap::_Polygon_default_instance_);
}
inline ::apollo::hdmap::Polygon* SpeedControl::release_polygon() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.SpeedControl.polygon)
  _has_bits_[0] &= ~0x00000002u;
  ::apollo::hdmap::Polygon* temp = polygon_;
  polygon_ = nullptr;
  return temp;
}
inline ::apollo::hdmap::Polygon* SpeedControl::mutable_polygon() {
  _has_bits_[0] |= 0x00000002u;
  if (polygon_ == nullptr) {
    auto* p = CreateMaybeMessage<::apollo::hdmap::Polygon>(GetArenaNoVirtual());
    polygon_ = p;
  }
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.SpeedControl.polygon)
  return polygon_;
}
inline void SpeedControl::set_allocated_polygon(::apollo::hdmap::Polygon* polygon) {
  ::PROTOBUF_NAMESPACE_ID::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == nullptr) {
    delete reinterpret_cast< ::PROTOBUF_NAMESPACE_ID::MessageLite*>(polygon_);
  }
  if (polygon) {
    ::PROTOBUF_NAMESPACE_ID::Arena* submessage_arena = nullptr;
    if (message_arena != submessage_arena) {
      polygon = ::PROTOBUF_NAMESPACE_ID::internal::GetOwnedMessage(
          message_arena, polygon, submessage_arena);
    }
    _has_bits_[0] |= 0x00000002u;
  } else {
    _has_bits_[0] &= ~0x00000002u;
  }
  polygon_ = polygon;
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.SpeedControl.polygon)
}

// optional double speed_limit = 3;
inline bool SpeedControl::has_speed_limit() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void SpeedControl::clear_speed_limit() {
  speed_limit_ = 0;
  _has_bits_[0] &= ~0x00000004u;
}
inline double SpeedControl::speed_limit() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.SpeedControl.speed_limit)
  return speed_limit_;
}
inline void SpeedControl::set_speed_limit(double value) {
  _has_bits_[0] |= 0x00000004u;
  speed_limit_ = value;
  // @@protoc_insertion_point(field_set:apollo.hdmap.SpeedControl.speed_limit)
}

// -------------------------------------------------------------------

// SpeedControls

// repeated .apollo.hdmap.SpeedControl speed_control = 1;
inline int SpeedControls::speed_control_size() const {
  return speed_control_.size();
}
inline void SpeedControls::clear_speed_control() {
  speed_control_.Clear();
}
inline ::apollo::hdmap::SpeedControl* SpeedControls::mutable_speed_control(int index) {
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.SpeedControls.speed_control)
  return speed_control_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::SpeedControl >*
SpeedControls::mutable_speed_control() {
  // @@protoc_insertion_point(field_mutable_list:apollo.hdmap.SpeedControls.speed_control)
  return &speed_control_;
}
inline const ::apollo::hdmap::SpeedControl& SpeedControls::speed_control(int index) const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.SpeedControls.speed_control)
  return speed_control_.Get(index);
}
inline ::apollo::hdmap::SpeedControl* SpeedControls::add_speed_control() {
  // @@protoc_insertion_point(field_add:apollo.hdmap.SpeedControls.speed_control)
  return speed_control_.Add();
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::apollo::hdmap::SpeedControl >&
SpeedControls::speed_control() const {
  // @@protoc_insertion_point(field_list:apollo.hdmap.SpeedControls.speed_control)
  return speed_control_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fspeed_5fcontrol_2eproto
