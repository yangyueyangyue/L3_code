// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/error_code.proto

#include "modules/common/proto/error_code.pb.h"

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
namespace apollo {
namespace common {
class StatusPbDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<StatusPb> _instance;
} _StatusPb_default_instance_;
}  // namespace common
}  // namespace apollo
static void InitDefaultsscc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::_StatusPb_default_instance_;
    new (ptr) ::apollo::common::StatusPb();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::StatusPb::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, InitDefaultsscc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto}, {}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, error_code_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::StatusPb, msg_),
  1,
  0,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::apollo::common::StatusPb)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::common::_StatusPb_default_instance_),
};

const char descriptor_table_protodef_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n%modules/common/proto/error_code.proto\022"
  "\rapollo.common\"I\n\010StatusPb\0220\n\nerror_code"
  "\030\001 \001(\0162\030.apollo.common.ErrorCode:\002OK\022\013\n\003"
  "msg\030\002 \001(\t*\231\010\n\tErrorCode\022\006\n\002OK\020\000\022\022\n\rCONTR"
  "OL_ERROR\020\350\007\022\027\n\022CONTROL_INIT_ERROR\020\351\007\022\032\n\025"
  "CONTROL_COMPUTE_ERROR\020\352\007\022\030\n\023CONTROL_ESTO"
  "P_ERROR\020\353\007\022\032\n\025PERFECT_CONTROL_ERROR\020\354\007\022\021"
  "\n\014CANBUS_ERROR\020\320\017\022\032\n\025CAN_CLIENT_ERROR_BA"
  "SE\020\264\020\022(\n#CAN_CLIENT_ERROR_OPEN_DEVICE_FA"
  "ILED\020\265\020\022\037\n\032CAN_CLIENT_ERROR_FRAME_NUM\020\266\020"
  "\022!\n\034CAN_CLIENT_ERROR_SEND_FAILED\020\267\020\022!\n\034C"
  "AN_CLIENT_ERROR_RECV_FAILED\020\270\020\022\027\n\022LOCALI"
  "ZATION_ERROR\020\270\027\022\033\n\026LOCALIZATION_ERROR_MS"
  "G\020\234\030\022\035\n\030LOCALIZATION_ERROR_LIDAR\020\200\031\022\035\n\030L"
  "OCALIZATION_ERROR_INTEG\020\344\031\022\034\n\027LOCALIZATI"
  "ON_ERROR_GNSS\020\310\032\022\025\n\020PERCEPTION_ERROR\020\240\037\022"
  "\030\n\023PERCEPTION_ERROR_TF\020\241\037\022\035\n\030PERCEPTION_"
  "ERROR_PROCESS\020\242\037\022\025\n\020PERCEPTION_FATAL\020\243\037\022"
  "\032\n\025PERCEPTION_ERROR_NONE\020\244\037\022\035\n\030PERCEPTIO"
  "N_ERROR_UNKNOWN\020\245\037\022\025\n\020PREDICTION_ERROR\020\210"
  "\'\022\023\n\016PLANNING_ERROR\020\360.\022\035\n\030PLANNING_ERROR"
  "_NOT_READY\020\361.\022\025\n\020HDMAP_DATA_ERROR\020\3306\022\022\n\r"
  "ROUTING_ERROR\020\300>\022\032\n\025ROUTING_ERROR_REQUES"
  "T\020\301>\022\033\n\026ROUTING_ERROR_RESPONSE\020\302>\022\034\n\027ROU"
  "TING_ERROR_NOT_READY\020\303>\022\021\n\014END_OF_INPUT\020"
  "\250F\022\025\n\020HTTP_LOGIC_ERROR\020\220N\022\027\n\022HTTP_RUNTIM"
  "E_ERROR\020\221N\022\027\n\022RELATIVE_MAP_ERROR\020\370U\022\033\n\026R"
  "ELATIVE_MAP_NOT_READY\020\371U\022\026\n\021DRIVER_ERROR"
  "_GNSS\020\340]\022\032\n\025DRIVER_ERROR_VELODYNE\020\310e\022\027\n\022"
  "STORYTELLING_ERROR\020\260m"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_sccs[1] = {
  &scc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_once;
static bool descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto = {
  &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_initialized, descriptor_table_protodef_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, "modules/common/proto/error_code.proto", 1181,
  &descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_once, descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_sccs, descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto_deps, 1, 0,
  schemas, file_default_instances, TableStruct_modules_2fcommon_2fproto_2ferror_5fcode_2eproto::offsets,
  file_level_metadata_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, 1, file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto, file_level_service_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fcommon_2fproto_2ferror_5fcode_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto), true);
namespace apollo {
namespace common {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* ErrorCode_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fcommon_2fproto_2ferror_5fcode_2eproto);
  return file_level_enum_descriptors_modules_2fcommon_2fproto_2ferror_5fcode_2eproto[0];
}
bool ErrorCode_IsValid(int value) {
  switch (value) {
    case 0:
    case 1000:
    case 1001:
    case 1002:
    case 1003:
    case 1004:
    case 2000:
    case 2100:
    case 2101:
    case 2102:
    case 2103:
    case 2104:
    case 3000:
    case 3100:
    case 3200:
    case 3300:
    case 3400:
    case 4000:
    case 4001:
    case 4002:
    case 4003:
    case 4004:
    case 4005:
    case 5000:
    case 6000:
    case 6001:
    case 7000:
    case 8000:
    case 8001:
    case 8002:
    case 8003:
    case 9000:
    case 10000:
    case 10001:
    case 11000:
    case 11001:
    case 12000:
    case 13000:
    case 14000:
      return true;
    default:
      return false;
  }
}


// ===================================================================

void StatusPb::InitAsDefaultInstance() {
}
class StatusPb::_Internal {
 public:
  using HasBits = decltype(std::declval<StatusPb>()._has_bits_);
  static void set_has_error_code(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_msg(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

StatusPb::StatusPb()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.StatusPb)
}
StatusPb::StatusPb(const StatusPb& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  msg_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from.has_msg()) {
    msg_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.msg_);
  }
  error_code_ = from.error_code_;
  // @@protoc_insertion_point(copy_constructor:apollo.common.StatusPb)
}

void StatusPb::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto.base);
  msg_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  error_code_ = 0;
}

StatusPb::~StatusPb() {
  // @@protoc_insertion_point(destructor:apollo.common.StatusPb)
  SharedDtor();
}

void StatusPb::SharedDtor() {
  msg_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void StatusPb::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const StatusPb& StatusPb::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_StatusPb_modules_2fcommon_2fproto_2ferror_5fcode_2eproto.base);
  return *internal_default_instance();
}


void StatusPb::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.StatusPb)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    msg_.ClearNonDefaultToEmptyNoArena();
  }
  error_code_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* StatusPb::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 8)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::common::ErrorCode_IsValid(val))) {
            set_error_code(static_cast<::apollo::common::ErrorCode>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(1, val, mutable_unknown_fields());
          }
        } else goto handle_unusual;
        continue;
      // optional string msg = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(mutable_msg(), ptr, ctx, "apollo.common.StatusPb.msg");
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
bool StatusPb::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.StatusPb)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (8 & 0xFF)) {
          int value = 0;
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   int, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::common::ErrorCode_IsValid(value)) {
            set_error_code(static_cast< ::apollo::common::ErrorCode >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                1, static_cast<::PROTOBUF_NAMESPACE_ID::uint64>(value));
          }
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string msg = 2;
      case 2: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (18 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadString(
                input, this->mutable_msg()));
          ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
            this->msg().data(), static_cast<int>(this->msg().length()),
            ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::PARSE,
            "apollo.common.StatusPb.msg");
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
  // @@protoc_insertion_point(parse_success:apollo.common.StatusPb)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.StatusPb)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void StatusPb::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.StatusPb)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnum(
      1, this->error_code(), output);
  }

  // optional string msg = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->msg().data(), static_cast<int>(this->msg().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.common.StatusPb.msg");
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->msg(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.StatusPb)
}

::PROTOBUF_NAMESPACE_ID::uint8* StatusPb::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.StatusPb)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
  if (cached_has_bits & 0x00000002u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      1, this->error_code(), target);
  }

  // optional string msg = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->msg().data(), static_cast<int>(this->msg().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.common.StatusPb.msg");
    target =
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringToArray(
        2, this->msg(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.StatusPb)
  return target;
}

size_t StatusPb::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.StatusPb)
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
  if (cached_has_bits & 0x00000003u) {
    // optional string msg = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->msg());
    }

    // optional .apollo.common.ErrorCode error_code = 1 [default = OK];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->error_code());
    }

  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void StatusPb::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.StatusPb)
  GOOGLE_DCHECK_NE(&from, this);
  const StatusPb* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<StatusPb>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.StatusPb)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.StatusPb)
    MergeFrom(*source);
  }
}

void StatusPb::MergeFrom(const StatusPb& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.StatusPb)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      msg_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.msg_);
    }
    if (cached_has_bits & 0x00000002u) {
      error_code_ = from.error_code_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void StatusPb::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.StatusPb)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void StatusPb::CopyFrom(const StatusPb& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.StatusPb)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool StatusPb::IsInitialized() const {
  return true;
}

void StatusPb::InternalSwap(StatusPb* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  msg_.Swap(&other->msg_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(error_code_, other->error_code_);
}

::PROTOBUF_NAMESPACE_ID::Metadata StatusPb::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::common::StatusPb* Arena::CreateMaybeMessage< ::apollo::common::StatusPb >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::StatusPb >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
