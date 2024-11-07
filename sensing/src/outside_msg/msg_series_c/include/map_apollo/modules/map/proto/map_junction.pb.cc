// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: modules/map/proto/map_junction.proto

#include "modules/map/proto/map_junction.pb.h"

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
extern PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fid_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_Id_modules_2fmap_2fproto_2fmap_5fid_2eproto;
extern PROTOBUF_INTERNAL_EXPORT_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_Polygon_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto;
namespace apollo {
namespace hdmap {
class JunctionDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<Junction> _instance;
} _Junction_default_instance_;
}  // namespace hdmap
}  // namespace apollo
static void InitDefaultsscc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::apollo::hdmap::_Junction_default_instance_;
    new (ptr) ::apollo::hdmap::Junction();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::apollo::hdmap::Junction::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<2> scc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 2, InitDefaultsscc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto}, {
      &scc_info_Id_modules_2fmap_2fproto_2fmap_5fid_2eproto.base,
      &scc_info_Polygon_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_modules_2fmap_2fproto_2fmap_5fjunction_2eproto[1];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fjunction_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fjunction_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_modules_2fmap_2fproto_2fmap_5fjunction_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, polygon_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, overlap_id_),
  PROTOBUF_FIELD_OFFSET(::apollo::hdmap::Junction, type_),
  0,
  1,
  ~0u,
  2,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 9, sizeof(::apollo::hdmap::Junction)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::apollo::hdmap::_Junction_default_instance_),
};

const char descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fjunction_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n$modules/map/proto/map_junction.proto\022\014"
  "apollo.hdmap\032\036modules/map/proto/map_id.p"
  "roto\032$modules/map/proto/map_geometry.pro"
  "to\"\377\001\n\010Junction\022\034\n\002id\030\001 \001(\0132\020.apollo.hdm"
  "ap.Id\022&\n\007polygon\030\002 \001(\0132\025.apollo.hdmap.Po"
  "lygon\022$\n\noverlap_id\030\003 \003(\0132\020.apollo.hdmap"
  ".Id\022)\n\004type\030\004 \001(\0162\033.apollo.hdmap.Junctio"
  "n.Type\"\\\n\004Type\022\013\n\007UNKNOWN\020\000\022\013\n\007IN_ROAD\020\001"
  "\022\016\n\nCROSS_ROAD\020\002\022\r\n\tFORK_ROAD\020\003\022\r\n\tMAIN_"
  "SIDE\020\004\022\014\n\010DEAD_END\020\005"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_deps[2] = {
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fgeometry_2eproto,
  &::descriptor_table_modules_2fmap_2fproto_2fmap_5fid_2eproto,
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_sccs[1] = {
  &scc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_once;
static bool descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_initialized = false;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto = {
  &descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_initialized, descriptor_table_protodef_modules_2fmap_2fproto_2fmap_5fjunction_2eproto, "modules/map/proto/map_junction.proto", 380,
  &descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_once, descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_sccs, descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto_deps, 1, 2,
  schemas, file_default_instances, TableStruct_modules_2fmap_2fproto_2fmap_5fjunction_2eproto::offsets,
  file_level_metadata_modules_2fmap_2fproto_2fmap_5fjunction_2eproto, 1, file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fjunction_2eproto, file_level_service_descriptors_modules_2fmap_2fproto_2fmap_5fjunction_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_modules_2fmap_2fproto_2fmap_5fjunction_2eproto = (  ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto), true);
namespace apollo {
namespace hdmap {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Junction_Type_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_modules_2fmap_2fproto_2fmap_5fjunction_2eproto);
  return file_level_enum_descriptors_modules_2fmap_2fproto_2fmap_5fjunction_2eproto[0];
}
bool Junction_Type_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)
constexpr Junction_Type Junction::UNKNOWN;
constexpr Junction_Type Junction::IN_ROAD;
constexpr Junction_Type Junction::CROSS_ROAD;
constexpr Junction_Type Junction::FORK_ROAD;
constexpr Junction_Type Junction::MAIN_SIDE;
constexpr Junction_Type Junction::DEAD_END;
constexpr Junction_Type Junction::Type_MIN;
constexpr Junction_Type Junction::Type_MAX;
constexpr int Junction::Type_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || _MSC_VER >= 1900)

// ===================================================================

void Junction::InitAsDefaultInstance() {
  ::apollo::hdmap::_Junction_default_instance_._instance.get_mutable()->id_ = const_cast< ::apollo::hdmap::Id*>(
      ::apollo::hdmap::Id::internal_default_instance());
  ::apollo::hdmap::_Junction_default_instance_._instance.get_mutable()->polygon_ = const_cast< ::apollo::hdmap::Polygon*>(
      ::apollo::hdmap::Polygon::internal_default_instance());
}
class Junction::_Internal {
 public:
  using HasBits = decltype(std::declval<Junction>()._has_bits_);
  static const ::apollo::hdmap::Id& id(const Junction* msg);
  static void set_has_id(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static const ::apollo::hdmap::Polygon& polygon(const Junction* msg);
  static void set_has_polygon(HasBits* has_bits) {
    (*has_bits)[0] |= 2u;
  }
  static void set_has_type(HasBits* has_bits) {
    (*has_bits)[0] |= 4u;
  }
};

const ::apollo::hdmap::Id&
Junction::_Internal::id(const Junction* msg) {
  return *msg->id_;
}
const ::apollo::hdmap::Polygon&
Junction::_Internal::polygon(const Junction* msg) {
  return *msg->polygon_;
}
void Junction::clear_id() {
  if (id_ != nullptr) id_->Clear();
  _has_bits_[0] &= ~0x00000001u;
}
void Junction::clear_polygon() {
  if (polygon_ != nullptr) polygon_->Clear();
  _has_bits_[0] &= ~0x00000002u;
}
void Junction::clear_overlap_id() {
  overlap_id_.Clear();
}
Junction::Junction()
  : ::PROTOBUF_NAMESPACE_ID::Message(), _internal_metadata_(nullptr) {
  SharedCtor();
  // @@protoc_insertion_point(constructor:apollo.hdmap.Junction)
}
Junction::Junction(const Junction& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _internal_metadata_(nullptr),
      _has_bits_(from._has_bits_),
      overlap_id_(from.overlap_id_) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_id()) {
    id_ = new ::apollo::hdmap::Id(*from.id_);
  } else {
    id_ = nullptr;
  }
  if (from.has_polygon()) {
    polygon_ = new ::apollo::hdmap::Polygon(*from.polygon_);
  } else {
    polygon_ = nullptr;
  }
  type_ = from.type_;
  // @@protoc_insertion_point(copy_constructor:apollo.hdmap.Junction)
}

void Junction::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto.base);
  ::memset(&id_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&type_) -
      reinterpret_cast<char*>(&id_)) + sizeof(type_));
}

Junction::~Junction() {
  // @@protoc_insertion_point(destructor:apollo.hdmap.Junction)
  SharedDtor();
}

void Junction::SharedDtor() {
  if (this != internal_default_instance()) delete id_;
  if (this != internal_default_instance()) delete polygon_;
}

void Junction::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const Junction& Junction::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_Junction_modules_2fmap_2fproto_2fmap_5fjunction_2eproto.base);
  return *internal_default_instance();
}


void Junction::Clear() {
// @@protoc_insertion_point(message_clear_start:apollo.hdmap.Junction)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  overlap_id_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000003u) {
    if (cached_has_bits & 0x00000001u) {
      GOOGLE_DCHECK(id_ != nullptr);
      id_->Clear();
    }
    if (cached_has_bits & 0x00000002u) {
      GOOGLE_DCHECK(polygon_ != nullptr);
      polygon_->Clear();
    }
  }
  type_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear();
}

#if GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER
const char* Junction::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // optional .apollo.hdmap.Id id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          ptr = ctx->ParseMessage(mutable_id(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // optional .apollo.hdmap.Polygon polygon = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr = ctx->ParseMessage(mutable_polygon(), ptr);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 26)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(add_overlap_id(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<::PROTOBUF_NAMESPACE_ID::uint8>(ptr) == 26);
        } else goto handle_unusual;
        continue;
      // optional .apollo.hdmap.Junction.Type type = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 32)) {
          ::PROTOBUF_NAMESPACE_ID::uint64 val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint(&ptr);
          CHK_(ptr);
          if (PROTOBUF_PREDICT_TRUE(::apollo::hdmap::Junction_Type_IsValid(val))) {
            set_type(static_cast<::apollo::hdmap::Junction_Type>(val));
          } else {
            ::PROTOBUF_NAMESPACE_ID::internal::WriteVarint(4, val, mutable_unknown_fields());
          }
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
bool Junction::MergePartialFromCodedStream(
    ::PROTOBUF_NAMESPACE_ID::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!PROTOBUF_PREDICT_TRUE(EXPRESSION)) goto failure
  ::PROTOBUF_NAMESPACE_ID::uint32 tag;
  // @@protoc_insertion_point(parse_start:apollo.hdmap.Junction)
  for (;;) {
    ::std::pair<::PROTOBUF_NAMESPACE_ID::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // optional .apollo.hdmap.Id id = 1;
      case 1: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (10 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
               input, mutable_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Polygon polygon = 2;
      case 2: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (18 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
               input, mutable_polygon()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // repeated .apollo.hdmap.Id overlap_id = 3;
      case 3: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (26 & 0xFF)) {
          DO_(::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadMessage(
                input, add_overlap_id()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // optional .apollo.hdmap.Junction.Type type = 4;
      case 4: {
        if (static_cast< ::PROTOBUF_NAMESPACE_ID::uint8>(tag) == (32 & 0xFF)) {
          int value = 0;
          DO_((::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::ReadPrimitive<
                   int, ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::TYPE_ENUM>(
                 input, &value)));
          if (::apollo::hdmap::Junction_Type_IsValid(value)) {
            set_type(static_cast< ::apollo::hdmap::Junction_Type >(value));
          } else {
            mutable_unknown_fields()->AddVarint(
                4, static_cast<::PROTOBUF_NAMESPACE_ID::uint64>(value));
          }
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
  // @@protoc_insertion_point(parse_success:apollo.hdmap.Junction)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:apollo.hdmap.Junction)
  return false;
#undef DO_
}
#endif  // GOOGLE_PROTOBUF_ENABLE_EXPERIMENTAL_PARSER

void Junction::SerializeWithCachedSizes(
    ::PROTOBUF_NAMESPACE_ID::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:apollo.hdmap.Junction)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, _Internal::id(this), output);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, _Internal::polygon(this), output);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteMessageMaybeToArray(
      3,
      this->overlap_id(static_cast<int>(i)),
      output);
  }

  // optional .apollo.hdmap.Junction.Type type = 4;
  if (cached_has_bits & 0x00000004u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnum(
      4, this->type(), output);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFields(
        _internal_metadata_.unknown_fields(), output);
  }
  // @@protoc_insertion_point(serialize_end:apollo.hdmap.Junction)
}

::PROTOBUF_NAMESPACE_ID::uint8* Junction::InternalSerializeWithCachedSizesToArray(
    ::PROTOBUF_NAMESPACE_ID::uint8* target) const {
  // @@protoc_insertion_point(serialize_to_array_start:apollo.hdmap.Junction)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // optional .apollo.hdmap.Id id = 1;
  if (cached_has_bits & 0x00000001u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, _Internal::id(this), target);
  }

  // optional .apollo.hdmap.Polygon polygon = 2;
  if (cached_has_bits & 0x00000002u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, _Internal::polygon(this), target);
  }

  // repeated .apollo.hdmap.Id overlap_id = 3;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->overlap_id_size()); i < n; i++) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, this->overlap_id(static_cast<int>(i)), target);
  }

  // optional .apollo.hdmap.Junction.Type type = 4;
  if (cached_has_bits & 0x00000004u) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      4, this->type(), target);
  }

  if (_internal_metadata_.have_unknown_fields()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields(), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:apollo.hdmap.Junction)
  return target;
}

size_t Junction::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:apollo.hdmap.Junction)
  size_t total_size = 0;

  if (_internal_metadata_.have_unknown_fields()) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::ComputeUnknownFieldsSize(
        _internal_metadata_.unknown_fields());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .apollo.hdmap.Id overlap_id = 3;
  {
    unsigned int count = static_cast<unsigned int>(this->overlap_id_size());
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          this->overlap_id(static_cast<int>(i)));
    }
  }

  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    // optional .apollo.hdmap.Id id = 1;
    if (cached_has_bits & 0x00000001u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *id_);
    }

    // optional .apollo.hdmap.Polygon polygon = 2;
    if (cached_has_bits & 0x00000002u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
          *polygon_);
    }

    // optional .apollo.hdmap.Junction.Type type = 4;
    if (cached_has_bits & 0x00000004u) {
      total_size += 1 +
        ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->type());
    }

  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void Junction::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:apollo.hdmap.Junction)
  GOOGLE_DCHECK_NE(&from, this);
  const Junction* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<Junction>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:apollo.hdmap.Junction)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:apollo.hdmap.Junction)
    MergeFrom(*source);
  }
}

void Junction::MergeFrom(const Junction& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:apollo.hdmap.Junction)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  overlap_id_.MergeFrom(from.overlap_id_);
  cached_has_bits = from._has_bits_[0];
  if (cached_has_bits & 0x00000007u) {
    if (cached_has_bits & 0x00000001u) {
      mutable_id()->::apollo::hdmap::Id::MergeFrom(from.id());
    }
    if (cached_has_bits & 0x00000002u) {
      mutable_polygon()->::apollo::hdmap::Polygon::MergeFrom(from.polygon());
    }
    if (cached_has_bits & 0x00000004u) {
      type_ = from.type_;
    }
    _has_bits_[0] |= cached_has_bits;
  }
}

void Junction::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:apollo.hdmap.Junction)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Junction::CopyFrom(const Junction& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:apollo.hdmap.Junction)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Junction::IsInitialized() const {
  return true;
}

void Junction::InternalSwap(Junction* other) {
  using std::swap;
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  CastToBase(&overlap_id_)->InternalSwap(CastToBase(&other->overlap_id_));
  swap(id_, other->id_);
  swap(polygon_, other->polygon_);
  swap(type_, other->type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Junction::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace hdmap
}  // namespace apollo
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::apollo::hdmap::Junction* Arena::CreateMaybeMessage< ::apollo::hdmap::Junction >(Arena* arena) {
  return Arena::CreateInternal< ::apollo::hdmap::Junction >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
