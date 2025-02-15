// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_id.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fid_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fid_2eproto

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
#define PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fid_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_modules_2fmap_2fproto_2fmap_5fid_2eproto {
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
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto;
namespace apollo {
namespace hdmap {
class Id;
class IdDefaultTypeInternal;
extern IdDefaultTypeInternal _Id_default_instance_;
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> ::apollo::hdmap::Id* Arena::CreateMaybeMessage<::apollo::hdmap::Id>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace apollo {
namespace hdmap {

// ===================================================================

class Id :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:apollo.hdmap.Id) */ {
 public:
  Id();
  virtual ~Id();

  Id(const Id& from);
  Id(Id&& from) noexcept
    : Id() {
    *this = ::std::move(from);
  }

  inline Id& operator=(const Id& from) {
    CopyFrom(from);
    return *this;
  }
  inline Id& operator=(Id&& from) noexcept {
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
  static const Id& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Id* internal_default_instance() {
    return reinterpret_cast<const Id*>(
               &_Id_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Id& a, Id& b) {
    a.Swap(&b);
  }
  inline void Swap(Id* other) {
    if (other == this) return;
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  inline Id* New() const final {
    return CreateMaybeMessage<Id>(nullptr);
  }

  Id* New(::PROTOBUF_NAMESPACE_ID::Arena* arena) const final {
    return CreateMaybeMessage<Id>(arena);
  }
  void CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) final;
  void CopyFrom(const Id& from);
  void MergeFrom(const Id& from);
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
  void InternalSwap(Id* other);
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "apollo.hdmap.Id";
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
    ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto);
    return ::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto.file_level_metadata[kIndexInFileMessages];
  }

  public:

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kIdFieldNumber = 1,
  };
  // optional string id = 1;
  bool has_id() const;
  void clear_id();
  const std::string& id() const;
  void set_id(const std::string& value);
  void set_id(std::string&& value);
  void set_id(const char* value);
  void set_id(const char* value, size_t size);
  std::string* mutable_id();
  std::string* release_id();
  void set_allocated_id(std::string* id);

  // @@protoc_insertion_point(class_scope:apollo.hdmap.Id)
 private:
  class _Internal;

  ::PROTOBUF_NAMESPACE_ID::internal::InternalMetadataWithArena _internal_metadata_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr id_;
  friend struct ::TableStruct_modules_2fmap_2fproto_2fmap_5fid_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Id

// optional string id = 1;
inline bool Id::has_id() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void Id::clear_id() {
  id_.ClearToEmptyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& Id::id() const {
  // @@protoc_insertion_point(field_get:apollo.hdmap.Id.id)
  return id_.GetNoArena();
}
inline void Id::set_id(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), value);
  // @@protoc_insertion_point(field_set:apollo.hdmap.Id.id)
}
inline void Id::set_id(std::string&& value) {
  _has_bits_[0] |= 0x00000001u;
  id_.SetNoArena(
    &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::move(value));
  // @@protoc_insertion_point(field_set_rvalue:apollo.hdmap.Id.id)
}
inline void Id::set_id(const char* value) {
  GOOGLE_DCHECK(value != nullptr);
  _has_bits_[0] |= 0x00000001u;
  id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), ::std::string(value));
  // @@protoc_insertion_point(field_set_char:apollo.hdmap.Id.id)
}
inline void Id::set_id(const char* value, size_t size) {
  _has_bits_[0] |= 0x00000001u;
  id_.SetNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      ::std::string(reinterpret_cast<const char*>(value), size));
  // @@protoc_insertion_point(field_set_pointer:apollo.hdmap.Id.id)
}
inline std::string* Id::mutable_id() {
  _has_bits_[0] |= 0x00000001u;
  // @@protoc_insertion_point(field_mutable:apollo.hdmap.Id.id)
  return id_.MutableNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline std::string* Id::release_id() {
  // @@protoc_insertion_point(field_release:apollo.hdmap.Id.id)
  if (!has_id()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  return id_.ReleaseNonDefaultNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}
inline void Id::set_allocated_id(std::string* id) {
  if (id != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  id_.SetAllocatedNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), id);
  // @@protoc_insertion_point(field_set_allocated:apollo.hdmap.Id.id)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace hdmap
}  // namespace apollo

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_modules_2fmap_2fproto_2fmap_5fid_2eproto
