// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: sphere.proto

#include "sphere.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG

namespace _pb = ::PROTOBUF_NAMESPACE_ID;
namespace _pbi = _pb::internal;

namespace autonomous_proto {
PROTOBUF_CONSTEXPR Sphere_Raw::Sphere_Raw(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.int64_array_)*/{}
  , /*decltype(_impl_._int64_array_cached_byte_size_)*/{0}
  , /*decltype(_impl_.float64_array_)*/{}
  , /*decltype(_impl_.name_)*/nullptr
  , /*decltype(_impl_.int64_value_)*/nullptr
  , /*decltype(_impl_.float64_value_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct Sphere_RawDefaultTypeInternal {
  PROTOBUF_CONSTEXPR Sphere_RawDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~Sphere_RawDefaultTypeInternal() {}
  union {
    Sphere_Raw _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 Sphere_RawDefaultTypeInternal _Sphere_Raw_default_instance_;
PROTOBUF_CONSTEXPR Sphere::Sphere(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.data_)*/{}
  , /*decltype(_impl_.header_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct SphereDefaultTypeInternal {
  PROTOBUF_CONSTEXPR SphereDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~SphereDefaultTypeInternal() {}
  union {
    Sphere _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 SphereDefaultTypeInternal _Sphere_default_instance_;
}  // namespace autonomous_proto
static ::_pb::Metadata file_level_metadata_sphere_2eproto[2];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_sphere_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_sphere_2eproto = nullptr;

const uint32_t TableStruct_sphere_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _impl_.name_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _impl_.int64_value_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _impl_.float64_value_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _impl_.int64_array_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere_Raw, _impl_.float64_array_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere, _impl_.header_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::Sphere, _impl_.data_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::autonomous_proto::Sphere_Raw)},
  { 11, -1, -1, sizeof(::autonomous_proto::Sphere)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::autonomous_proto::_Sphere_Raw_default_instance_._instance,
  &::autonomous_proto::_Sphere_default_instance_._instance,
};

const char descriptor_table_protodef_sphere_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\014sphere.proto\022\020autonomous_proto\032\036google"
  "/protobuf/wrappers.proto\032\022message_info.p"
  "roto\"\252\002\n\006Sphere\022-\n\006header\030\001 \001(\0132\035.autono"
  "mous_proto.MessageInfo\022*\n\004data\030\002 \003(\0132\034.a"
  "utonomous_proto.Sphere.Raw\032\304\001\n\003Raw\022*\n\004na"
  "me\030\001 \001(\0132\034.google.protobuf.StringValue\0220"
  "\n\013int64_value\030\002 \001(\0132\033.google.protobuf.In"
  "t64Value\0223\n\rfloat64_value\030\003 \001(\0132\034.google"
  ".protobuf.DoubleValue\022\023\n\013int64_array\030\004 \003"
  "(\003\022\025\n\rfloat64_array\030\005 \003(\001b\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_sphere_2eproto_deps[2] = {
  &::descriptor_table_google_2fprotobuf_2fwrappers_2eproto,
  &::descriptor_table_message_5finfo_2eproto,
};
static ::_pbi::once_flag descriptor_table_sphere_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_sphere_2eproto = {
    false, false, 393, descriptor_table_protodef_sphere_2eproto,
    "sphere.proto",
    &descriptor_table_sphere_2eproto_once, descriptor_table_sphere_2eproto_deps, 2, 2,
    schemas, file_default_instances, TableStruct_sphere_2eproto::offsets,
    file_level_metadata_sphere_2eproto, file_level_enum_descriptors_sphere_2eproto,
    file_level_service_descriptors_sphere_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_sphere_2eproto_getter() {
  return &descriptor_table_sphere_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_sphere_2eproto(&descriptor_table_sphere_2eproto);
namespace autonomous_proto {

// ===================================================================

class Sphere_Raw::_Internal {
 public:
  static const ::PROTOBUF_NAMESPACE_ID::StringValue& name(const Sphere_Raw* msg);
  static const ::PROTOBUF_NAMESPACE_ID::Int64Value& int64_value(const Sphere_Raw* msg);
  static const ::PROTOBUF_NAMESPACE_ID::DoubleValue& float64_value(const Sphere_Raw* msg);
};

const ::PROTOBUF_NAMESPACE_ID::StringValue&
Sphere_Raw::_Internal::name(const Sphere_Raw* msg) {
  return *msg->_impl_.name_;
}
const ::PROTOBUF_NAMESPACE_ID::Int64Value&
Sphere_Raw::_Internal::int64_value(const Sphere_Raw* msg) {
  return *msg->_impl_.int64_value_;
}
const ::PROTOBUF_NAMESPACE_ID::DoubleValue&
Sphere_Raw::_Internal::float64_value(const Sphere_Raw* msg) {
  return *msg->_impl_.float64_value_;
}
void Sphere_Raw::clear_name() {
  if (GetArenaForAllocation() == nullptr && _impl_.name_ != nullptr) {
    delete _impl_.name_;
  }
  _impl_.name_ = nullptr;
}
void Sphere_Raw::clear_int64_value() {
  if (GetArenaForAllocation() == nullptr && _impl_.int64_value_ != nullptr) {
    delete _impl_.int64_value_;
  }
  _impl_.int64_value_ = nullptr;
}
void Sphere_Raw::clear_float64_value() {
  if (GetArenaForAllocation() == nullptr && _impl_.float64_value_ != nullptr) {
    delete _impl_.float64_value_;
  }
  _impl_.float64_value_ = nullptr;
}
Sphere_Raw::Sphere_Raw(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.Sphere.Raw)
}
Sphere_Raw::Sphere_Raw(const Sphere_Raw& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  Sphere_Raw* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.int64_array_){from._impl_.int64_array_}
    , /*decltype(_impl_._int64_array_cached_byte_size_)*/{0}
    , decltype(_impl_.float64_array_){from._impl_.float64_array_}
    , decltype(_impl_.name_){nullptr}
    , decltype(_impl_.int64_value_){nullptr}
    , decltype(_impl_.float64_value_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_name()) {
    _this->_impl_.name_ = new ::PROTOBUF_NAMESPACE_ID::StringValue(*from._impl_.name_);
  }
  if (from._internal_has_int64_value()) {
    _this->_impl_.int64_value_ = new ::PROTOBUF_NAMESPACE_ID::Int64Value(*from._impl_.int64_value_);
  }
  if (from._internal_has_float64_value()) {
    _this->_impl_.float64_value_ = new ::PROTOBUF_NAMESPACE_ID::DoubleValue(*from._impl_.float64_value_);
  }
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.Sphere.Raw)
}

inline void Sphere_Raw::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.int64_array_){arena}
    , /*decltype(_impl_._int64_array_cached_byte_size_)*/{0}
    , decltype(_impl_.float64_array_){arena}
    , decltype(_impl_.name_){nullptr}
    , decltype(_impl_.int64_value_){nullptr}
    , decltype(_impl_.float64_value_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

Sphere_Raw::~Sphere_Raw() {
  // @@protoc_insertion_point(destructor:autonomous_proto.Sphere.Raw)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Sphere_Raw::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.int64_array_.~RepeatedField();
  _impl_.float64_array_.~RepeatedField();
  if (this != internal_default_instance()) delete _impl_.name_;
  if (this != internal_default_instance()) delete _impl_.int64_value_;
  if (this != internal_default_instance()) delete _impl_.float64_value_;
}

void Sphere_Raw::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void Sphere_Raw::Clear() {
// @@protoc_insertion_point(message_clear_start:autonomous_proto.Sphere.Raw)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.int64_array_.Clear();
  _impl_.float64_array_.Clear();
  if (GetArenaForAllocation() == nullptr && _impl_.name_ != nullptr) {
    delete _impl_.name_;
  }
  _impl_.name_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.int64_value_ != nullptr) {
    delete _impl_.int64_value_;
  }
  _impl_.int64_value_ = nullptr;
  if (GetArenaForAllocation() == nullptr && _impl_.float64_value_ != nullptr) {
    delete _impl_.float64_value_;
  }
  _impl_.float64_value_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Sphere_Raw::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .google.protobuf.StringValue name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_name(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .google.protobuf.Int64Value int64_value = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr = ctx->ParseMessage(_internal_mutable_int64_value(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // .google.protobuf.DoubleValue float64_value = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          ptr = ctx->ParseMessage(_internal_mutable_float64_value(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated int64 int64_array = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedInt64Parser(_internal_mutable_int64_array(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 32) {
          _internal_add_int64_array(::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated double float64_array = 5;
      case 5:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 42)) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_float64_array(), ptr, ctx);
          CHK_(ptr);
        } else if (static_cast<uint8_t>(tag) == 41) {
          _internal_add_float64_array(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
          ptr += sizeof(double);
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Sphere_Raw::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autonomous_proto.Sphere.Raw)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .google.protobuf.StringValue name = 1;
  if (this->_internal_has_name()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::name(this),
        _Internal::name(this).GetCachedSize(), target, stream);
  }

  // .google.protobuf.Int64Value int64_value = 2;
  if (this->_internal_has_int64_value()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, _Internal::int64_value(this),
        _Internal::int64_value(this).GetCachedSize(), target, stream);
  }

  // .google.protobuf.DoubleValue float64_value = 3;
  if (this->_internal_has_float64_value()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(3, _Internal::float64_value(this),
        _Internal::float64_value(this).GetCachedSize(), target, stream);
  }

  // repeated int64 int64_array = 4;
  {
    int byte_size = _impl_._int64_array_cached_byte_size_.load(std::memory_order_relaxed);
    if (byte_size > 0) {
      target = stream->WriteInt64Packed(
          4, _internal_int64_array(), byte_size, target);
    }
  }

  // repeated double float64_array = 5;
  if (this->_internal_float64_array_size() > 0) {
    target = stream->WriteFixedPacked(5, _internal_float64_array(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autonomous_proto.Sphere.Raw)
  return target;
}

size_t Sphere_Raw::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autonomous_proto.Sphere.Raw)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated int64 int64_array = 4;
  {
    size_t data_size = ::_pbi::WireFormatLite::
      Int64Size(this->_impl_.int64_array_);
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    int cached_size = ::_pbi::ToCachedSize(data_size);
    _impl_._int64_array_cached_byte_size_.store(cached_size,
                                    std::memory_order_relaxed);
    total_size += data_size;
  }

  // repeated double float64_array = 5;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_float64_array_size());
    size_t data_size = 8UL * count;
    if (data_size > 0) {
      total_size += 1 +
        ::_pbi::WireFormatLite::Int32Size(static_cast<int32_t>(data_size));
    }
    total_size += data_size;
  }

  // .google.protobuf.StringValue name = 1;
  if (this->_internal_has_name()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.name_);
  }

  // .google.protobuf.Int64Value int64_value = 2;
  if (this->_internal_has_int64_value()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.int64_value_);
  }

  // .google.protobuf.DoubleValue float64_value = 3;
  if (this->_internal_has_float64_value()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.float64_value_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Sphere_Raw::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    Sphere_Raw::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Sphere_Raw::GetClassData() const { return &_class_data_; }


void Sphere_Raw::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<Sphere_Raw*>(&to_msg);
  auto& from = static_cast<const Sphere_Raw&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:autonomous_proto.Sphere.Raw)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.int64_array_.MergeFrom(from._impl_.int64_array_);
  _this->_impl_.float64_array_.MergeFrom(from._impl_.float64_array_);
  if (from._internal_has_name()) {
    _this->_internal_mutable_name()->::PROTOBUF_NAMESPACE_ID::StringValue::MergeFrom(
        from._internal_name());
  }
  if (from._internal_has_int64_value()) {
    _this->_internal_mutable_int64_value()->::PROTOBUF_NAMESPACE_ID::Int64Value::MergeFrom(
        from._internal_int64_value());
  }
  if (from._internal_has_float64_value()) {
    _this->_internal_mutable_float64_value()->::PROTOBUF_NAMESPACE_ID::DoubleValue::MergeFrom(
        from._internal_float64_value());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Sphere_Raw::CopyFrom(const Sphere_Raw& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autonomous_proto.Sphere.Raw)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Sphere_Raw::IsInitialized() const {
  return true;
}

void Sphere_Raw::InternalSwap(Sphere_Raw* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.int64_array_.InternalSwap(&other->_impl_.int64_array_);
  _impl_.float64_array_.InternalSwap(&other->_impl_.float64_array_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Sphere_Raw, _impl_.float64_value_)
      + sizeof(Sphere_Raw::_impl_.float64_value_)
      - PROTOBUF_FIELD_OFFSET(Sphere_Raw, _impl_.name_)>(
          reinterpret_cast<char*>(&_impl_.name_),
          reinterpret_cast<char*>(&other->_impl_.name_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Sphere_Raw::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_sphere_2eproto_getter, &descriptor_table_sphere_2eproto_once,
      file_level_metadata_sphere_2eproto[0]);
}

// ===================================================================

class Sphere::_Internal {
 public:
  static const ::autonomous_proto::MessageInfo& header(const Sphere* msg);
};

const ::autonomous_proto::MessageInfo&
Sphere::_Internal::header(const Sphere* msg) {
  return *msg->_impl_.header_;
}
void Sphere::clear_header() {
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
}
Sphere::Sphere(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.Sphere)
}
Sphere::Sphere(const Sphere& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  Sphere* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.data_){from._impl_.data_}
    , decltype(_impl_.header_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    _this->_impl_.header_ = new ::autonomous_proto::MessageInfo(*from._impl_.header_);
  }
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.Sphere)
}

inline void Sphere::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.data_){arena}
    , decltype(_impl_.header_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

Sphere::~Sphere() {
  // @@protoc_insertion_point(destructor:autonomous_proto.Sphere)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void Sphere::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.data_.~RepeatedPtrField();
  if (this != internal_default_instance()) delete _impl_.header_;
}

void Sphere::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void Sphere::Clear() {
// @@protoc_insertion_point(message_clear_start:autonomous_proto.Sphere)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.data_.Clear();
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Sphere::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::_pbi::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // .autonomous_proto.MessageInfo header = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          ptr = ctx->ParseMessage(_internal_mutable_header(), ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .autonomous_proto.Sphere.Raw data = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_data(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Sphere::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autonomous_proto.Sphere)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .autonomous_proto.MessageInfo header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // repeated .autonomous_proto.Sphere.Raw data = 2;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_data_size()); i < n; i++) {
    const auto& repfield = this->_internal_data(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(2, repfield, repfield.GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autonomous_proto.Sphere)
  return target;
}

size_t Sphere::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autonomous_proto.Sphere)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .autonomous_proto.Sphere.Raw data = 2;
  total_size += 1UL * this->_internal_data_size();
  for (const auto& msg : this->_impl_.data_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // .autonomous_proto.MessageInfo header = 1;
  if (this->_internal_has_header()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(
        *_impl_.header_);
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_impl_._cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Sphere::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    Sphere::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Sphere::GetClassData() const { return &_class_data_; }


void Sphere::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<Sphere*>(&to_msg);
  auto& from = static_cast<const Sphere&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:autonomous_proto.Sphere)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.data_.MergeFrom(from._impl_.data_);
  if (from._internal_has_header()) {
    _this->_internal_mutable_header()->::autonomous_proto::MessageInfo::MergeFrom(
        from._internal_header());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Sphere::CopyFrom(const Sphere& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autonomous_proto.Sphere)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Sphere::IsInitialized() const {
  return true;
}

void Sphere::InternalSwap(Sphere* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.data_.InternalSwap(&other->_impl_.data_);
  swap(_impl_.header_, other->_impl_.header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Sphere::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_sphere_2eproto_getter, &descriptor_table_sphere_2eproto_once,
      file_level_metadata_sphere_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace autonomous_proto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::autonomous_proto::Sphere_Raw*
Arena::CreateMaybeMessage< ::autonomous_proto::Sphere_Raw >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::Sphere_Raw >(arena);
}
template<> PROTOBUF_NOINLINE ::autonomous_proto::Sphere*
Arena::CreateMaybeMessage< ::autonomous_proto::Sphere >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::Sphere >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
