// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: chassis_ft_auman.proto

#include "chassis_ft_auman.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
namespace phoenix {
namespace msg {
namespace control {
class ChassisFtAumanDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<ChassisFtAuman> _instance;
} _ChassisFtAuman_default_instance_;
}  // namespace control
}  // namespace msg
}  // namespace phoenix
static void InitDefaultsscc_info_ChassisFtAuman_chassis_5fft_5fauman_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::phoenix::msg::control::_ChassisFtAuman_default_instance_;
    new (ptr) ::phoenix::msg::control::ChassisFtAuman();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::phoenix::msg::control::ChassisFtAuman::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_ChassisFtAuman_chassis_5fft_5fauman_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsscc_info_ChassisFtAuman_chassis_5fft_5fauman_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_chassis_5fft_5fauman_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_chassis_5fft_5fauman_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_chassis_5fft_5fauman_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_chassis_5fft_5fauman_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::phoenix::msg::control::ChassisFtAuman, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::phoenix::msg::control::ChassisFtAuman, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::phoenix::msg::control::ChassisFtAuman, switch_tja_),
  PROTOBUF_FIELD_OFFSET(::phoenix::msg::control::ChassisFtAuman, switch_hwa_),
  PROTOBUF_FIELD_OFFSET(::phoenix::msg::control::ChassisFtAuman, switch_i_drive_),
  0,
  1,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, sizeof(::phoenix::msg::control::ChassisFtAuman)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::phoenix::msg::control::_ChassisFtAuman_default_instance_),
};

const char descriptor_table_protodef_chassis_5fft_5fauman_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\026chassis_ft_auman.proto\022\023phoenix.msg.co"
  "ntrol\"Y\n\016ChassisFtAuman\022\025\n\nswitch_tja\030\001 "
  "\001(\005:\0010\022\025\n\nswitch_hwa\030\002 \001(\005:\0010\022\031\n\016switch_"
  "i_drive\030\003 \001(\005:\0010"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_chassis_5fft_5fauman_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_chassis_5fft_5fauman_2eproto_sccs[1] = {
  &scc_info_ChassisFtAuman_chassis_5fft_5fauman_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_chassis_5fft_5fauman_2eproto_once;
static bool descriptor_table_chassis_5fft_5fauman_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_chassis_5fft_5fauman_2eproto = {
  &descriptor_table_chassis_5fft_5fauman_2eproto_initialized, descriptor_table_protodef_chassis_5fft_5fauman_2eproto, "chassis_ft_auman.proto", 136,
  &descriptor_table_chassis_5fft_5fauman_2eproto_once, descriptor_table_chassis_5fft_5fauman_2eproto_sccs, descriptor_table_chassis_5fft_5fauman_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_chassis_5fft_5fauman_2eproto::offsets,
  file_level_metadata_chassis_5fft_5fauman_2eproto, 1, file_level_enum_descriptors_chassis_5fft_5fauman_2eproto, file_level_service_descriptors_chassis_5fft_5fauman_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_chassis_5fft_5fauman_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_chassis_5fft_5fauman_2eproto), true);
namespace phoenix {
namespace msg {
namespace control {

// ===================================================================

void ChassisFtAuman::InitAsDefaultInstance() {
}
class ChassisFtAuman::_Internal {
 public:
  using HasBits = decltype(std::declval<ChassisFtAuman>()._has_bits_);
  static void set_has_switch_tja(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_switch_hwa(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_switch_i_drive(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

ChassisFtAuman::ChassisFtAuman()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:phoenix.msg.control.ChassisFtAuman)
}
ChassisFtAuman::ChassisFtAuman(const ChassisFtAuman& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&switch_tja_, &from.switch_tja_,
    static_cast<size_t>(reinterpret_cast<char*>(&switch_i_drive_) -
    reinterpret_cast<char*>(&switch_tja_)) + sizeof(switch_i_drive_));
  // @@protoc_insertion_point(copy_constructor:phoenix.msg.control.ChassisFtAuman)
}

void ChassisFtAuman::SharedCtor() {
  ::memset(&switch_tja_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&switch_i_drive_) -
      reinterpret_cast<char*>(&switch_tja_)) + sizeof(switch_i_drive_));
}

ChassisFtAuman::~ChassisFtAuman() {
  // @@protoc_insertion_point(destructor:phoenix.msg.control.ChassisFtAuman)
  SharedDtor();
}

void ChassisFtAuman::SharedDtor() {
}

void ChassisFtAuman::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const ChassisFtAuman& ChassisFtAuman::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_ChassisFtAuman_chassis_5fft_5fauman_2eproto.base);
  return *internal_default_instance();
}


void ChassisFtAuman::Clear() {
// @@protoc_insertion_point(message_clear_start:phoenix.msg.control.ChassisFtAuman)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    ::memset(&switch_tja_, 0, static_cast<size_t>(
        reinterpret_cast<char*>(&switch_i_drive_) -
        reinterpret_cast<char*>(&switch_tja_)) + sizeof(switch_i_drive_));
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* ChassisFtAuman::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional int32 switch_tja = 1 [default = 0];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          _Internal::set_has_switch_tja(&has_bits);
          switch_tja_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 switch_hwa = 2 [default = 0];
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 16)) {
          _Internal::set_has_switch_hwa(&has_bits);
          switch_hwa_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional int32 switch_i_drive = 3 [default = 0];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 24)) {
          _Internal::set_has_switch_i_drive(&has_bits);
          switch_i_drive_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag, &_internal_metadata_, ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool ChassisFtAuman::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:phoenix.msg.control.ChassisFtAuman)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional int32 switch_tja = 1 [default = 0];
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (8 & 0xFF)) {
          _Internal::set_has_switch_tja(&_has_bits_);
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   ::PROTOBUF_NAMESPACE_ID::int32, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_INT32>(
                 input, &switch_tja_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 switch_hwa = 2 [default = 0];
      case 2: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (16 & 0xFF)) {
          _Internal::set_has_switch_hwa(&_has_bits_);
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   ::PROTOBUF_NAMESPACE_ID::int32, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_INT32>(
                 input, &switch_hwa_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional int32 switch_i_drive = 3 [default = 0];
      case 3: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (24 & 0xFF)) {
          _Internal::set_has_switch_i_drive(&_has_bits_);
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   ::PROTOBUF_NAMESPACE_ID::int32, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_INT32>(
                 input, &switch_i_drive_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:phoenix.msg.control.ChassisFtAuman)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:phoenix.msg.control.ChassisFtAuman)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void ChassisFtAuman::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:phoenix.msg.control.ChassisFtAuman)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 switch_tja = 1 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32(1, this->switch_tja(), output);
  }

  // optional int32 switch_hwa = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32(2, this->switch_hwa(), output);
  }

  // optional int32 switch_i_drive = 3 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32(3, this->switch_i_drive(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:phoenix.msg.control.ChassisFtAuman)
}

::PROTOBUF_NAMESPACE_ID::uint8* ChassisFtAuman::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:phoenix.msg.control.ChassisFtAuman)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional int32 switch_tja = 1 [default = 0];
  if (cached_has_bits & 0x00000001u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(1, this->switch_tja(), target);
  }

  // optional int32 switch_hwa = 2 [default = 0];
  if (cached_has_bits & 0x00000002u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(2, this->switch_hwa(), target);
  }

  // optional int32 switch_i_drive = 3 [default = 0];
  if (cached_has_bits & 0x00000004u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteInt32ToArray(3, this->switch_i_drive(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:phoenix.msg.control.ChassisFtAuman)
  return target;
}

size_t ChassisFtAuman::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:phoenix.msg.control.ChassisFtAuman)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional int32 switch_tja = 1 [default = 0];
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->switch_tja());
    }

    // optional int32 switch_hwa = 2 [default = 0];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->switch_hwa());
    }

    // optional int32 switch_i_drive = 3 [default = 0];
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::Int32Size(
          this->switch_i_drive());
    }

  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void ChassisFtAuman::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:phoenix.msg.control.ChassisFtAuman)
  GOOGLE_DCHECK_NE(&from, this);
  const ChassisFtAuman* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<ChassisFtAuman>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:phoenix.msg.control.ChassisFtAuman)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:phoenix.msg.control.ChassisFtAuman)
    MergeFrom(*source);
  }
}

void ChassisFtAuman::MergeFrom(const ChassisFtAuman& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:phoenix.msg.control.ChassisFtAuman)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      switch_tja_ = from.switch_tja_;
    }
    if (cached_has_bits & 0x00000002u) {
      switch_hwa_ = from.switch_hwa_;
    }
    if (cached_has_bits & 0x00000004u) {
      switch_i_drive_ = from.switch_i_drive_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void ChassisFtAuman::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:phoenix.msg.control.ChassisFtAuman)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void ChassisFtAuman::CopyFrom(const ChassisFtAuman& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:phoenix.msg.control.ChassisFtAuman)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool ChassisFtAuman::IsInitialized() const {
  return true;
}

void ChassisFtAuman::InternalSwap(ChassisFtAuman* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  swap(switch_tja_, other->switch_tja_);
  swap(switch_hwa_, other->switch_hwa_);
  swap(switch_i_drive_, other->switch_i_drive_);
}

::PROTOBUF_NAMESPACE_ID::Metadata ChassisFtAuman::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace control
}  // namespace msg
}  // namespace phoenix
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::phoenix::msg::control::ChassisFtAuman* Arena::CreateMaybeMessage< ::phoenix::msg::control::ChassisFtAuman >(Arena* arena) {
  return Arena::CreateInternal< ::phoenix::msg::control::ChassisFtAuman >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
