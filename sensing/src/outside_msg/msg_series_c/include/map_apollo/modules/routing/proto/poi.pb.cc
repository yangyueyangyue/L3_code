// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/routing/proto/poi.proto

#include "modules/routing/proto/poi.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2frouting_2fproto_2fpoi_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2frouting_2fproto_2frouting_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_LaneWaypoint_modules_2frouting_2fproto_2frouting_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2frouting_2fproto_2frouting_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_ParkingInfo_modules_2frouting_2fproto_2frouting_2eproto;
namespace apollo {
namespace routing {
class LandmarkDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Landmark> _instance;
} _Landmark_default_instance_;
class POIDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<POI> _instance;
} _POI_default_instance_;
}  // namespace routing
}  // namespace apollo
static void InitDefaultsscc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::routing::_Landmark_default_instance_;
    new (ptr) ::apollo::routing::Landmark();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::routing::Landmark::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsscc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto}, {
      &scc_info_LaneWaypoint_modules_2frouting_2fproto_2frouting_2eproto.base,
      &scc_info_ParkingInfo_modules_2frouting_2fproto_2frouting_2eproto.base,}};

static void InitDefaultsscc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::routing::_POI_default_instance_;
    new (ptr) ::apollo::routing::POI();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::routing::POI::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, InitDefaultsscc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto}, {
      &scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2frouting_2fproto_2fpoi_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_modules_2frouting_2fproto_2fpoi_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2frouting_2fproto_2fpoi_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2frouting_2fproto_2fpoi_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, name_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, waypoint_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, parking_space_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::Landmark, parking_info_),
  0,
  ~0u,
  1,
  2,
  PROTOBUF_FIELD_OFFSET(::apollo::routing::POI, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::routing::POI, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::routing::POI, landmark_),
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::routing::Landmark)},
  { 13, 19, sizeof(::apollo::routing::POI)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::routing::_Landmark_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::routing::_POI_default_instance_),
};

const char descriptor_table_protodef_modules_2frouting_2fproto_2fpoi_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\037modules/routing/proto/poi.proto\022\016apoll"
  "o.routing\032#modules/routing/proto/routing"
  ".proto\"\231\001\n\010Landmark\022\014\n\004name\030\001 \001(\t\022.\n\010way"
  "point\030\002 \003(\0132\034.apollo.routing.LaneWaypoin"
  "t\022\034\n\020parking_space_id\030\003 \001(\tB\002\030\001\0221\n\014parki"
  "ng_info\030\004 \001(\0132\033.apollo.routing.ParkingIn"
  "fo\"1\n\003POI\022*\n\010landmark\030\001 \003(\0132\030.apollo.rou"
  "ting.Landmark"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_deps[1] = {
  &::descriptor_table_modules_2frouting_2fproto_2frouting_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_sccs[2] = {
  &scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto.base,
  &scc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_once;
static bool descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto = {
  &descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_initialized, descriptor_table_protodef_modules_2frouting_2fproto_2fpoi_2eproto, "modules/routing/proto/poi.proto", 293,
  &descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_once, descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_sccs, descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto_deps, 2, 1,
  schemas, file_default_instances, TableStruct_modules_2frouting_2fproto_2fpoi_2eproto::offsets,
  file_level_metadata_modules_2frouting_2fproto_2fpoi_2eproto, 2, file_level_enum_descriptors_modules_2frouting_2fproto_2fpoi_2eproto, file_level_service_descriptors_modules_2frouting_2fproto_2fpoi_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2frouting_2fproto_2fpoi_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2frouting_2fproto_2fpoi_2eproto), true);
namespace apollo {
namespace routing {

// ===================================================================

void Landmark::InitAsDefaultInstance() {
  ::apollo::routing::_Landmark_default_instance_._instance.get_mutable()->parking_info_ = const_cast< ::apollo::routing::ParkingInfo*>(
      ::apollo::routing::ParkingInfo::internal_default_instance());
}
class Landmark::_Internal {
 public:
  using HasBits = decltype(std::declval<Landmark>()._has_bits_);
  static void set_has_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static void set_has_parking_space_id(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static const ::apollo::routing::ParkingInfo& parking_info(const Landmark* msg);
  static void set_has_parking_info(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::routing::ParkingInfo&
Landmark::_Internal::parking_info(const Landmark* msg) {
  return *msg->parking_info_;
}
void Landmark::clear_waypoint() {
  waypoint_.Clear();
}
void Landmark::clear_parking_info() {
  if (parking_info_ != nullptr) parking_info_->Clear();
  _has_bits_[0] &= ~0x00000004u;
}
Landmark::Landmark()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.routing.Landmark)
}
Landmark::Landmark(const Landmark& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      waypoint_(from.waypoint_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from.has_name()) {
    name_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.name_);
  }
  parking_space_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from.has_parking_space_id()) {
    parking_space_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.parking_space_id_);
  }
  if (from.has_parking_info()) {
    parking_info_ = new ::apollo::routing::ParkingInfo(*from.parking_info_);
  } else {
    parking_info_ = nullptr;
  }
  // @@protoc_insertion_point(copy_constructor:apollo.routing.Landmark)
}

void Landmark::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto.base);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  parking_space_id_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  parking_info_ = nullptr;
}

Landmark::~Landmark() {
  // @@protoc_insertion_point(destructor:apollo.routing.Landmark)
  SharedDtor();
}

void Landmark::SharedDtor() {
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  parking_space_id_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (this != internal_default_instance()) delete parking_info_;
}

void Landmark::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Landmark& Landmark::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Landmark_modules_2frouting_2fproto_2fpoi_2eproto.base);
  return *internal_default_instance();
}


void Landmark::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.routing.Landmark)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  waypoint_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      name_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000002u) {
      parking_space_id_.ClearNonDefaultToEmptyNoArena();
    }
    if (cached_has_bits & 0x00000004u) {
      GOOGLE_DCHECK(parking_info_ != nullptr);
      parking_info_->Clear();
    }
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Landmark::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional string name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(mutable_name(), ptr, ctx, "apollo.routing.Landmark.name");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.routing.LaneWaypoint waypoint = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(add_waypoint(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<::PROTOBUF_NAMESPACE_ID::uint8>(ptr) == 18);
        } else goto handle_unusual;
        continue;
      // optional string parking_space_id = 3 [deprecated = true];
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParserUTF8Verify(mutable_parking_space_id(), ptr, ctx, "apollo.routing.Landmark.parking_space_id");
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.routing.ParkingInfo parking_info = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 34)) {
          ptr = ctx->ParseMessage(mutable_parking_info(), ptr);
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
bool Landmark::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.routing.Landmark)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional string name = 1;
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (10 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadString(
                input, this->mutable_name()));
          ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
            this->name().data(), static_cast<int>(this->name().length()),
            ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::PARSE,
            "apollo.routing.Landmark.name");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.routing.LaneWaypoint waypoint = 2;
      case 2: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (18 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
                input, add_waypoint()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional string parking_space_id = 3 [deprecated = true];
      case 3: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (26 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadString(
                input, this->mutable_parking_space_id()));
          ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
            this->parking_space_id().data(), static_cast<int>(this->parking_space_id().length()),
            ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::PARSE,
            "apollo.routing.Landmark.parking_space_id");
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.routing.ParkingInfo parking_info = 4;
      case 4: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (34 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
               input, mutable_parking_info()));
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
  // @@protoc_insertion_point(parse_success:apollo.routing.Landmark)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.routing.Landmark)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Landmark::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.routing.Landmark)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string name = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), static_cast<int>(this->name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.Landmark.name");
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringMaybeAliased(
      1, this->name(), output);
  }

  // repeated .apollo.routing.LaneWaypoint waypoint = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->waypoint_size()); i < n; i++) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      2,
      this->waypoint(static_cast<int>(i)),
      output);
  }

  // optional string parking_space_id = 3 [deprecated = true];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parking_space_id().data(), static_cast<int>(this->parking_space_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.Landmark.parking_space_id");
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringMaybeAliased(
      3, this->parking_space_id(), output);
  }

  // optional .apollo.routing.ParkingInfo parking_info = 4;
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, _Internal::parking_info(this), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.routing.Landmark)
}

::PROTOBUF_NAMESPACE_ID::uint8* Landmark::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.routing.Landmark)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional string name = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->name().data(), static_cast<int>(this->name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.Landmark.name");
    target =
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringToArray(
        1, this->name(), target);
  }

  // repeated .apollo.routing.LaneWaypoint waypoint = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->waypoint_size()); i < n; i++) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, this->waypoint(static_cast<int>(i)), target);
  }

  // optional string parking_space_id = 3 [deprecated = true];
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->parking_space_id().data(), static_cast<int>(this->parking_space_id().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "apollo.routing.Landmark.parking_space_id");
    target =
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteStringToArray(
        3, this->parking_space_id(), target);
  }

  // optional .apollo.routing.ParkingInfo parking_info = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, _Internal::parking_info(this), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.routing.Landmark)
  return target;
}

size_t Landmark::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.routing.Landmark)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.routing.LaneWaypoint waypoint = 2;
  {
    unsigned int count = static_cast<unsigned int>(this->waypoint_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          this->waypoint(static_cast<int>(i)));
    }
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional string name = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->name());
    }

    // optional string parking_space_id = 3 [deprecated = true];
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
          this->parking_space_id());
    }

    // optional .apollo.routing.ParkingInfo parking_info = 4;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *parking_info_);
    }

  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Landmark::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.routing.Landmark)
  GOOGLE_DCHECK_NE(&from, this);
  const Landmark* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Landmark>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.routing.Landmark)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.routing.Landmark)
    MergeFrom(*source);
  }
}

void Landmark::MergeFrom(const Landmark& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.routing.Landmark)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  waypoint_.MergeFrom(from.waypoint_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      _has_bits_[0] |= 0x00000001u;
      name_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.name_);
    }
    if (cached_has_bits & 0x00000002u) {
      _has_bits_[0] |= 0x00000002u;
      parking_space_id_.AssignWithDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from.parking_space_id_);
    }
    if (cached_has_bits & 0x00000004u) {
      mutable_parking_info()->::apollo::routing::ParkingInfo::MergeFrom(from.parking_info());
    }
  }
}

void Landmark::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.routing.Landmark)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Landmark::CopyFrom(const Landmark& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.routing.Landmark)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Landmark::IsInitialized() const {
  return true;
}

void Landmark::InternalSwap(Landmark* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  CastToBase(&waypoint_)->InternalSwap(CastToBase(&other->waypoint_));
  name_.Swap(&other->name_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  parking_space_id_.Swap(&other->parking_space_id_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
    GetArenaNoVirtual());
  swap(parking_info_, other->parking_info_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Landmark::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void POI::InitAsDefaultInstance() {
}
class POI::_Internal {
 public:
  using HasBits = decltype(std::declval<POI>()._has_bits_);
};

POI::POI()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.routing.POI)
}
POI::POI(const POI& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      landmark_(from.landmark_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  // @@protoc_insertion_point(copy_constructor:apollo.routing.POI)
}

void POI::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto.base);
}

POI::~POI() {
  // @@protoc_insertion_point(destructor:apollo.routing.POI)
  SharedDtor();
}

void POI::SharedDtor() {
}

void POI::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const POI& POI::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_POI_modules_2frouting_2fproto_2fpoi_2eproto.base);
  return *internal_default_instance();
}


void POI::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.routing.POI)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  landmark_.Clear();
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* POI::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated .apollo.routing.Landmark landmark = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(add_landmark(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<::PROTOBUF_NAMESPACE_ID::uint8>(ptr) == 10);
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
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}
#else  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
bool POI::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.routing.POI)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .apollo.routing.Landmark landmark = 1;
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (10 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
                input, add_landmark()));
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
  // @@protoc_insertion_point(parse_success:apollo.routing.POI)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.routing.POI)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void POI::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.routing.POI)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.routing.Landmark landmark = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->landmark_size()); i < n; i++) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      1,
      this->landmark(static_cast<int>(i)),
      output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.routing.POI)
}

::PROTOBUF_NAMESPACE_ID::uint8* POI::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.routing.POI)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated .apollo.routing.Landmark landmark = 1;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->landmark_size()); i < n; i++) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, this->landmark(static_cast<int>(i)), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.routing.POI)
  return target;
}

size_t POI::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.routing.POI)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.routing.Landmark landmark = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->landmark_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          this->landmark(static_cast<int>(i)));
    }
  }

  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void POI::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.routing.POI)
  GOOGLE_DCHECK_NE(&from, this);
  const POI* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<POI>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.routing.POI)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.routing.POI)
    MergeFrom(*source);
  }
}

void POI::MergeFrom(const POI& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.routing.POI)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  landmark_.MergeFrom(from.landmark_);
}

void POI::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.routing.POI)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void POI::CopyFrom(const POI& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.routing.POI)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool POI::IsInitialized() const {
  return true;
}

void POI::InternalSwap(POI* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  CastToBase(&landmark_)->InternalSwap(CastToBase(&other->landmark_));
}

::PROTOBUF_NAMESPACE_ID::Metadata POI::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace routing
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::routing::Landmark* Arena::CreateMaybeMessage< ::apollo::routing::Landmark >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::routing::Landmark >(arena);
}
template<> PROTOBUF_NOINLINE ::apollo::routing::POI* Arena::CreateMaybeMessage< ::apollo::routing::POI >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::routing::POI >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
