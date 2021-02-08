// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto

#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {
class CeresScanMatcherOptions2DDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<CeresScanMatcherOptions2D>
      _instance;
} _CeresScanMatcherOptions2D_default_instance_;
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
namespace protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto {
void InitDefaultsCeresScanMatcherOptions2DImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  protobuf_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto::InitDefaultsCeresSolverOptions();
  {
    void* ptr = &::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions2D_default_instance_;
    new (ptr) ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D::InitAsDefaultInstance();
}

void InitDefaultsCeresScanMatcherOptions2D() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsCeresScanMatcherOptions2DImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D, occupied_space_weight_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D, translation_weight_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D, rotation_weight_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D, ceres_solver_options_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::cartographer::mapping::scan_matching::proto::CeresScanMatcherOptions2D)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions2D_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\nLcartographer/mapping/proto/scan_matchi"
      "ng/ceres_scan_matcher_options_2d.proto\022("
      "cartographer.mapping.scan_matching.proto"
      "\0324cartographer/common/proto/ceres_solver"
      "_options.proto\"\274\001\n\031CeresScanMatcherOptio"
      "ns2D\022\035\n\025occupied_space_weight\030\001 \001(\001\022\032\n\022t"
      "ranslation_weight\030\002 \001(\001\022\027\n\017rotation_weig"
      "ht\030\003 \001(\001\022K\n\024ceres_solver_options\030\t \001(\0132-"
      ".cartographer.common.proto.CeresSolverOp"
      "tionsb\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 373);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.proto", &protobuf_RegisterTypes);
  ::protobuf_cartographer_2fcommon_2fproto_2fceres_5fsolver_5foptions_2eproto::AddDescriptors();
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto
namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace proto {

// ===================================================================

void CeresScanMatcherOptions2D::InitAsDefaultInstance() {
  ::cartographer::mapping::scan_matching::proto::_CeresScanMatcherOptions2D_default_instance_._instance.get_mutable()->ceres_solver_options_ = const_cast< ::cartographer::common::proto::CeresSolverOptions*>(
      ::cartographer::common::proto::CeresSolverOptions::internal_default_instance());
}
void CeresScanMatcherOptions2D::clear_ceres_solver_options() {
  if (GetArenaNoVirtual() == NULL && ceres_solver_options_ != NULL) {
    delete ceres_solver_options_;
  }
  ceres_solver_options_ = NULL;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int CeresScanMatcherOptions2D::kOccupiedSpaceWeightFieldNumber;
const int CeresScanMatcherOptions2D::kTranslationWeightFieldNumber;
const int CeresScanMatcherOptions2D::kRotationWeightFieldNumber;
const int CeresScanMatcherOptions2D::kCeresSolverOptionsFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

CeresScanMatcherOptions2D::CeresScanMatcherOptions2D()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::InitDefaultsCeresScanMatcherOptions2D();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
}
CeresScanMatcherOptions2D::CeresScanMatcherOptions2D(const CeresScanMatcherOptions2D& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_ceres_solver_options()) {
    ceres_solver_options_ = new ::cartographer::common::proto::CeresSolverOptions(*from.ceres_solver_options_);
  } else {
    ceres_solver_options_ = NULL;
  }
  ::memcpy(&occupied_space_weight_, &from.occupied_space_weight_,
    static_cast<size_t>(reinterpret_cast<char*>(&rotation_weight_) -
    reinterpret_cast<char*>(&occupied_space_weight_)) + sizeof(rotation_weight_));
  // @@protoc_insertion_point(copy_constructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
}

void CeresScanMatcherOptions2D::SharedCtor() {
  ::memset(&ceres_solver_options_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_weight_) -
      reinterpret_cast<char*>(&ceres_solver_options_)) + sizeof(rotation_weight_));
  _cached_size_ = 0;
}

CeresScanMatcherOptions2D::~CeresScanMatcherOptions2D() {
  // @@protoc_insertion_point(destructor:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  SharedDtor();
}

void CeresScanMatcherOptions2D::SharedDtor() {
  if (this != internal_default_instance()) delete ceres_solver_options_;
}

void CeresScanMatcherOptions2D::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* CeresScanMatcherOptions2D::descriptor() {
  ::protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const CeresScanMatcherOptions2D& CeresScanMatcherOptions2D::default_instance() {
  ::protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::InitDefaultsCeresScanMatcherOptions2D();
  return *internal_default_instance();
}

CeresScanMatcherOptions2D* CeresScanMatcherOptions2D::New(::google::protobuf::Arena* arena) const {
  CeresScanMatcherOptions2D* n = new CeresScanMatcherOptions2D;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void CeresScanMatcherOptions2D::Clear() {
// @@protoc_insertion_point(message_clear_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == NULL && ceres_solver_options_ != NULL) {
    delete ceres_solver_options_;
  }
  ceres_solver_options_ = NULL;
  ::memset(&occupied_space_weight_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&rotation_weight_) -
      reinterpret_cast<char*>(&occupied_space_weight_)) + sizeof(rotation_weight_));
  _internal_metadata_.Clear();
}

bool CeresScanMatcherOptions2D::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double occupied_space_weight = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(9u /* 9 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &occupied_space_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double translation_weight = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(17u /* 17 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &translation_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double rotation_weight = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(25u /* 25 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &rotation_weight_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(74u /* 74 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_ceres_solver_options()));
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
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  return false;
#undef DO_
}

void CeresScanMatcherOptions2D::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double occupied_space_weight = 1;
  if (this->occupied_space_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->occupied_space_weight(), output);
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->translation_weight(), output);
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->rotation_weight(), output);
  }

  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 9;
  if (this->has_ceres_solver_options()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      9, *this->ceres_solver_options_, output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
}

::google::protobuf::uint8* CeresScanMatcherOptions2D::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double occupied_space_weight = 1;
  if (this->occupied_space_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->occupied_space_weight(), target);
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->translation_weight(), target);
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->rotation_weight(), target);
  }

  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 9;
  if (this->has_ceres_solver_options()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        9, *this->ceres_solver_options_, deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  return target;
}

size_t CeresScanMatcherOptions2D::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .cartographer.common.proto.CeresSolverOptions ceres_solver_options = 9;
  if (this->has_ceres_solver_options()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *this->ceres_solver_options_);
  }

  // double occupied_space_weight = 1;
  if (this->occupied_space_weight() != 0) {
    total_size += 1 + 8;
  }

  // double translation_weight = 2;
  if (this->translation_weight() != 0) {
    total_size += 1 + 8;
  }

  // double rotation_weight = 3;
  if (this->rotation_weight() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void CeresScanMatcherOptions2D::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  const CeresScanMatcherOptions2D* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const CeresScanMatcherOptions2D>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
    MergeFrom(*source);
  }
}

void CeresScanMatcherOptions2D::MergeFrom(const CeresScanMatcherOptions2D& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_ceres_solver_options()) {
    mutable_ceres_solver_options()->::cartographer::common::proto::CeresSolverOptions::MergeFrom(from.ceres_solver_options());
  }
  if (from.occupied_space_weight() != 0) {
    set_occupied_space_weight(from.occupied_space_weight());
  }
  if (from.translation_weight() != 0) {
    set_translation_weight(from.translation_weight());
  }
  if (from.rotation_weight() != 0) {
    set_rotation_weight(from.rotation_weight());
  }
}

void CeresScanMatcherOptions2D::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void CeresScanMatcherOptions2D::CopyFrom(const CeresScanMatcherOptions2D& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:cartographer.mapping.scan_matching.proto.CeresScanMatcherOptions2D)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool CeresScanMatcherOptions2D::IsInitialized() const {
  return true;
}

void CeresScanMatcherOptions2D::Swap(CeresScanMatcherOptions2D* other) {
  if (other == this) return;
  InternalSwap(other);
}
void CeresScanMatcherOptions2D::InternalSwap(CeresScanMatcherOptions2D* other) {
  using std::swap;
  swap(ceres_solver_options_, other->ceres_solver_options_);
  swap(occupied_space_weight_, other->occupied_space_weight_);
  swap(translation_weight_, other->translation_weight_);
  swap(rotation_weight_, other->rotation_weight_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata CeresScanMatcherOptions2D::GetMetadata() const {
  protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_cartographer_2fmapping_2fproto_2fscan_5fmatching_2fceres_5fscan_5fmatcher_5foptions_5f2d_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace proto
}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer

// @@protoc_insertion_point(global_scope)
