// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/common/proto/drive_event.proto

#include "modules/common/proto/drive_event.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fcommon_2fproto_2fheader_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2flocalization_2fproto_2fpose_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<3> scc_info_Pose_modules_2flocalization_2fproto_2fpose_2eproto;
namespace apollo {
namespace common {
class DriveEventDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<DriveEvent> _instance;
} _DriveEvent_default_instance_;
}  // namespace common
}  // namespace apollo
static void InitDefaultsscc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::common::_DriveEvent_default_instance_;
    new (ptr) ::apollo::common::DriveEvent();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::common::DriveEvent::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsscc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto}, {
      &scc_info_Header_modules_2fcommon_2fproto_2fheader_2eproto.base,
      &scc_info_Pose_modules_2flocalization_2fproto_2fpose_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, header_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, event_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, location_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, type_),
  PROTOBUF_FIELD_OFFSET(::apollo::common::DriveEvent, is_reportable_),
  1,
  0,
  2,
  ~0u,
  3,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 10, sizeof(::apollo::common::DriveEvent)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::common::_DriveEvent_default_instance_),
};

const char descriptor_table_protodef_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n&modules/common/proto/drive_event.proto"
  "\022\rapollo.common\032!modules/common/proto/he"
  "ader.proto\032%modules/localization/proto/p"
  "ose.proto\"\366\001\n\nDriveEvent\022%\n\006header\030\001 \001(\013"
  "2\025.apollo.common.Header\022\r\n\005event\030\002 \001(\t\022+"
  "\n\010location\030\003 \001(\0132\031.apollo.localization.P"
  "ose\022,\n\004type\030\004 \003(\0162\036.apollo.common.DriveE"
  "vent.Type\022\025\n\ris_reportable\030\005 \001(\010\"@\n\004Type"
  "\022\014\n\010CRITICAL\020\000\022\013\n\007PROBLEM\020\001\022\013\n\007DESIRED\020\002"
  "\022\020\n\014OUT_OF_SCOPE\020\003"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_deps[2] = {
  &::descriptor_table_modules_2fcommon_2fproto_2fheader_2eproto,
  &::descriptor_table_modules_2flocalization_2fproto_2fpose_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_sccs[1] = {
  &scc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_once;
static bool descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto = {
  &descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_initialized, descriptor_table_protodef_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto, "modules/common/proto/drive_event.proto", 378,
  &descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_once, descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_sccs, descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto::offsets,
  file_level_metadata_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto, 1, file_level_enum_descriptors_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto, file_level_service_descriptors_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto), true);
namespace apollo {
namespace common {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* DriveEvent_Type_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto);
  return file_level_enum_descriptors_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto[0];
}
bool DriveEvent_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr DriveEvent_Type DriveEvent::CRITICAL;
constexpr DriveEvent_Type DriveEvent::PROBLEM;
constexpr DriveEvent_Type DriveEvent::DESIRED;
constexpr DriveEvent_Type DriveEvent::OUT_OF_SCOPE;
constexpr DriveEvent_Type DriveEvent::Type_MIN;
constexpr DriveEvent_Type DriveEvent::Type_MAX;
constexpr int DriveEvent::Type_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

void DriveEvent::InitAsDefaultInstance() {
  ::apollo::common::_DriveEvent_default_instance_._instance.get_mutable()->header_ = const_cast< ::apollo::common::Header*>(
      ::apollo::common::Header::internal_default_instance());
  ::apollo::common::_DriveEvent_default_instance_._instance.get_mutable()->location_ = const_cast< ::apollo::localization::Pose*>(
      ::apollo::localization::Pose::internal_default_instance());
}
class DriveEvent::_Internal {
 public:
  using HasBits = decltype(std::declval<DriveEvent>()._has_bits_);
  static const ::apollo::common::Header& header(const DriveEvent* msg);
  static void set_has_header(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_event(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::localization::Pose& location(const DriveEvent* msg);
  static void set_has_location(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
  static void set_has_is_reportable(HasBits* has_bits) {
    (*has_bits)[0] |= 8u;
  }
};

const ::apollo::common::Header&
DriveEvent::_Internal::header(const DriveEvent* msg) {
  return *msg->header_;
}
const ::apollo::localization::Pose&
DriveEvent::_Internal::location(const DriveEvent* msg) {
  return *msg->location_;
}
void DriveEvent::clear_header() {
  if (header_ != nullptr) header_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void DriveEvent::clear_location() {
  if (location_ != nullptr) location_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
DriveEvent::DriveEvent()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.common.DriveEvent)
}
DriveEvent::DriveEvent(const DriveEvent& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      type_(from.type_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  event_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from.has_event()) {
    event_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.event_);
  }
  if (from.has_header()) {
    header_ = new ::apollo::common::Header(*from.header_);
  } else {
    header_ = nullptr;
  }
  if (from.has_location()) {
    location_ = new ::apollo::localization::Pose(*from.location_);
  } else {
    location_ = nullptr;
  }
  is_reportable_ = from.is_reportable_;
  // @@protoc_insertion_point(copy_constructor:apollo.common.DriveEvent)
}

void DriveEvent::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto.base);
  event_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&is_reportable_) -
      reinterpret_cast<char*>(&header_)) + sizeof(is_reportable_));
}

DriveEvent::~DriveEvent() {
  // @@protoc_insertion_point(destructor:apollo.common.DriveEvent)
  SharedDtor();
}

void DriveEvent::SharedDtor() {
  event_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete location_;
}

void DriveEvent::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const DriveEvent& DriveEvent::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_DriveEvent_modules_2fcommon_2fproto_2fdrive_5fevent_2eproto.base);
  return *internal_default_instance();
}


void DriveEvent::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.common.DriveEvent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  type_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      event_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(header_ != nullptr);
      header_->Clear();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(location_ != nullptr);
      location_->Clear();
    }
  }
  is_reportable_ = false;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* DriveEvent::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.common.Header header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(mutable_header(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional string event = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(mutable_event(), ptr, ctx, "apollo.common.DriveEvent.event");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.localization.Pose location = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ctx->ParseMessage(mutable_location(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.common.DriveEvent.Type type = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          ptr -= 1;
          do {
            ptr += 1;
            ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
            CHK_(ptr);
            if (PROTOBUF_PREDICT_TRUE(::apollo::common::DriveEvent_Type_IsValid(val))) {
              add_type(static_cast<::apollo::common::DriveEvent_Type>(val));
            } else {
              ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(4, val, mutable_unknown_fields());
            }
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<::PROTOBUF_NAMESPACE_ID::uint8>(ptr) == 32);
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedEnumParser(mutable_type(), ptr, ctx, ::apollo::common::DriveEvent_Type_IsValid, &_internal_metadata_, 4);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional bool is_reportable = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 40)) {
          _Internal::set_has_is_reportable(&has_bits);
          is_reportable_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
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
bool DriveEvent::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.common.DriveEvent)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.common.Header header = 1;
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (10 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string event = 2;
      case 2: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (18 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadString(
                input, this->mutable_event()));
          ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
            this->event().data(), static_cast<int>(this->event().length()),
            ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::PARSE,
            "apollo.common.DriveEvent.event");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.localization.Pose location = 3;
      case 3: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (26 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
               input, mutable_location()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.common.DriveEvent.Type type = 4;
      case 4: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (32 & 0xFF)) {
          int value = 0;
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   int, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::common::DriveEvent_Type_IsValid(value)) {
            add_type(static_cast< ::apollo::common::DriveEvent_Type >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                4, static_cast<::PROTOBUF_NAMESPACE_ID::uint64>(value));
          }
        } else if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (34 & 0xFF)) {
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ReadPackedEnumPreserveUnknowns(
                 input,
                 4,
                 ::apollo::common::DriveEvent_Type_IsValid,
                 mutable_unknown_fields(),
                 this->mutable_type())));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional bool is_reportable = 5;
      case 5: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (40 & 0xFF)) {
          _Internal::set_has_is_reportable(&_has_bits_);
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   bool, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_BOOL>(
                 input, &is_reportable_)));
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
  // @@protoc_insertion_point(parse_success:apollo.common.DriveEvent)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.common.DriveEvent)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void DriveEvent::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.common.DriveEvent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, _Internal::header(this), output);
  }

  // optional string event = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->event().data(), static_cast<int>(this->event().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.common.DriveEvent.event");
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringMaybeAliased(
      2, this->event(), output);
  }

  // optional .apollo.localization.Pose location = 3;
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, _Internal::location(this), output);
  }

  // repeated .apollo.common.DriveEvent.Type type = 4;
  for (int i = 0, n = this->type_size(); i < n; i++) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnum(
      4, this->type(i), output);
  }

  // optional bool is_reportable = 5;
  if (cached_has_bits & 0x00000008u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBool(5, this->is_reportable(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.common.DriveEvent)
}

::PROTOBUF_NAMESPACE_ID::uint8* DriveEvent::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.common.DriveEvent)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.common.Header header = 1;
  if (cached_has_bits & 0x00000002u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, _Internal::header(this), target);
  }

  // optional string event = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->event().data(), static_cast<int>(this->event().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.common.DriveEvent.event");
    target =
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringToArray(
        2, this->event(), target);
  }

  // optional .apollo.localization.Pose location = 3;
  if (cached_has_bits & 0x00000004u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, _Internal::location(this), target);
  }

  // repeated .apollo.common.DriveEvent.Type type = 4;
  target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
    4, this->type_, target);

  // optional bool is_reportable = 5;
  if (cached_has_bits & 0x00000008u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteBoolToArray(5, this->is_reportable(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.common.DriveEvent)
  return target;
}

size_t DriveEvent::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.common.DriveEvent)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.common.DriveEvent.Type type = 4;
  {
    size_t data_size = 0;
    unsigned int count = static_cast<unsigned int>(this->type_size());for (unsigned int i = 0; i < count; i++) {
      data_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(
        this->type(static_cast<int>(i)));
    }
    total_size += (1UL * count) + data_size;
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    // optional string event = 2;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->event());
    }

    // optional .apollo.common.Header header = 1;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *header_);
    }

    // optional .apollo.localization.Pose location = 3;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *location_);
    }

    // optional bool is_reportable = 5;
    if (cached_has_bits & 0x00000008u) {
      total_size += 1 + 1;
    }

  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void DriveEvent::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.common.DriveEvent)
  GOOGLE_DCHECK_NE(&from, this);
  const DriveEvent* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<DriveEvent>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.common.DriveEvent)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.common.DriveEvent)
    MergeFrom(*source);
  }
}

void DriveEvent::MergeFrom(const DriveEvent& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.common.DriveEvent)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  type_.MergeFrom(from.type_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x0000000fu) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      event_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.event_);
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_header()->::apollo::common::Header::MergeFrom(from.header());
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_location()->::apollo::localization::Pose::MergeFrom(from.location());
    }
    if (cached_has_bits & 0x00000008u) {
      is_reportable_ = from.is_reportable_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void DriveEvent::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.common.DriveEvent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void DriveEvent::CopyFrom(const DriveEvent& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.common.DriveEvent)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool DriveEvent::IsInitialized() const {
  return true;
}

void DriveEvent::InternalSwap(DriveEvent* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  type_.InternalSwap(&other->type_);
  event_.Swap(&other->event_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(header_, other->header_);
  swap(location_, other->location_);
  swap(is_reportable_, other->is_reportable_);
}

::PROTOBUF_NAMESPACE_ID::Metadata DriveEvent::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace common
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::common::DriveEvent* Arena::CreateMaybeMessage< ::apollo::common::DriveEvent >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::common::DriveEvent >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
