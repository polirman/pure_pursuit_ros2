// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: navigation_path.proto

#include "navigation_path.pb.h"

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
PROTOBUF_CONSTEXPR NavigationPath::NavigationPath(
    ::_pbi::ConstantInitialized): _impl_{
    /*decltype(_impl_.position_)*/{}
  , /*decltype(_impl_.header_)*/nullptr
  , /*decltype(_impl_._cached_size_)*/{}} {}
struct NavigationPathDefaultTypeInternal {
  PROTOBUF_CONSTEXPR NavigationPathDefaultTypeInternal()
      : _instance(::_pbi::ConstantInitialized{}) {}
  ~NavigationPathDefaultTypeInternal() {}
  union {
    NavigationPath _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT PROTOBUF_ATTRIBUTE_INIT_PRIORITY1 NavigationPathDefaultTypeInternal _NavigationPath_default_instance_;
}  // namespace autonomous_proto
static ::_pb::Metadata file_level_metadata_navigation_5fpath_2eproto[1];
static constexpr ::_pb::EnumDescriptor const** file_level_enum_descriptors_navigation_5fpath_2eproto = nullptr;
static constexpr ::_pb::ServiceDescriptor const** file_level_service_descriptors_navigation_5fpath_2eproto = nullptr;

const uint32_t TableStruct_navigation_5fpath_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::NavigationPath, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::NavigationPath, _impl_.header_),
  PROTOBUF_FIELD_OFFSET(::autonomous_proto::NavigationPath, _impl_.position_),
};
static const ::_pbi::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, -1, sizeof(::autonomous_proto::NavigationPath)},
};

static const ::_pb::Message* const file_default_instances[] = {
  &::autonomous_proto::_NavigationPath_default_instance_._instance,
};

const char descriptor_table_protodef_navigation_5fpath_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\025navigation_path.proto\022\020autonomous_prot"
  "o\032\022message_info.proto\032\020navigation.proto\""
  "x\n\016NavigationPath\022-\n\006header\030\001 \001(\0132\035.auto"
  "nomous_proto.MessageInfo\0227\n\010position\030\002 \003"
  "(\0132%.autonomous_proto.Navigation.Positio"
  "nb\006proto3"
  ;
static const ::_pbi::DescriptorTable* const descriptor_table_navigation_5fpath_2eproto_deps[2] = {
  &::descriptor_table_message_5finfo_2eproto,
  &::descriptor_table_navigation_2eproto,
};
static ::_pbi::once_flag descriptor_table_navigation_5fpath_2eproto_once;
const ::_pbi::DescriptorTable descriptor_table_navigation_5fpath_2eproto = {
    false, false, 209, descriptor_table_protodef_navigation_5fpath_2eproto,
    "navigation_path.proto",
    &descriptor_table_navigation_5fpath_2eproto_once, descriptor_table_navigation_5fpath_2eproto_deps, 2, 1,
    schemas, file_default_instances, TableStruct_navigation_5fpath_2eproto::offsets,
    file_level_metadata_navigation_5fpath_2eproto, file_level_enum_descriptors_navigation_5fpath_2eproto,
    file_level_service_descriptors_navigation_5fpath_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::_pbi::DescriptorTable* descriptor_table_navigation_5fpath_2eproto_getter() {
  return &descriptor_table_navigation_5fpath_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY2 static ::_pbi::AddDescriptorsRunner dynamic_init_dummy_navigation_5fpath_2eproto(&descriptor_table_navigation_5fpath_2eproto);
namespace autonomous_proto {

// ===================================================================

class NavigationPath::_Internal {
 public:
  static const ::autonomous_proto::MessageInfo& header(const NavigationPath* msg);
};

const ::autonomous_proto::MessageInfo&
NavigationPath::_Internal::header(const NavigationPath* msg) {
  return *msg->_impl_.header_;
}
void NavigationPath::clear_header() {
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
}
void NavigationPath::clear_position() {
  _impl_.position_.Clear();
}
NavigationPath::NavigationPath(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor(arena, is_message_owned);
  // @@protoc_insertion_point(arena_constructor:autonomous_proto.NavigationPath)
}
NavigationPath::NavigationPath(const NavigationPath& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  NavigationPath* const _this = this; (void)_this;
  new (&_impl_) Impl_{
      decltype(_impl_.position_){from._impl_.position_}
    , decltype(_impl_.header_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}};

  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  if (from._internal_has_header()) {
    _this->_impl_.header_ = new ::autonomous_proto::MessageInfo(*from._impl_.header_);
  }
  // @@protoc_insertion_point(copy_constructor:autonomous_proto.NavigationPath)
}

inline void NavigationPath::SharedCtor(
    ::_pb::Arena* arena, bool is_message_owned) {
  (void)arena;
  (void)is_message_owned;
  new (&_impl_) Impl_{
      decltype(_impl_.position_){arena}
    , decltype(_impl_.header_){nullptr}
    , /*decltype(_impl_._cached_size_)*/{}
  };
}

NavigationPath::~NavigationPath() {
  // @@protoc_insertion_point(destructor:autonomous_proto.NavigationPath)
  if (auto *arena = _internal_metadata_.DeleteReturnArena<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>()) {
  (void)arena;
    return;
  }
  SharedDtor();
}

inline void NavigationPath::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  _impl_.position_.~RepeatedPtrField();
  if (this != internal_default_instance()) delete _impl_.header_;
}

void NavigationPath::SetCachedSize(int size) const {
  _impl_._cached_size_.Set(size);
}

void NavigationPath::Clear() {
// @@protoc_insertion_point(message_clear_start:autonomous_proto.NavigationPath)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  _impl_.position_.Clear();
  if (GetArenaForAllocation() == nullptr && _impl_.header_ != nullptr) {
    delete _impl_.header_;
  }
  _impl_.header_ = nullptr;
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* NavigationPath::_InternalParse(const char* ptr, ::_pbi::ParseContext* ctx) {
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
      // repeated .autonomous_proto.Navigation.Position position = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_position(), ptr);
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

uint8_t* NavigationPath::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:autonomous_proto.NavigationPath)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // .autonomous_proto.MessageInfo header = 1;
  if (this->_internal_has_header()) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(1, _Internal::header(this),
        _Internal::header(this).GetCachedSize(), target, stream);
  }

  // repeated .autonomous_proto.Navigation.Position position = 2;
  for (unsigned i = 0,
      n = static_cast<unsigned>(this->_internal_position_size()); i < n; i++) {
    const auto& repfield = this->_internal_position(i);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
        InternalWriteMessage(2, repfield, repfield.GetCachedSize(), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::_pbi::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:autonomous_proto.NavigationPath)
  return target;
}

size_t NavigationPath::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:autonomous_proto.NavigationPath)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .autonomous_proto.Navigation.Position position = 2;
  total_size += 1UL * this->_internal_position_size();
  for (const auto& msg : this->_impl_.position_) {
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

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData NavigationPath::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSourceCheck,
    NavigationPath::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*NavigationPath::GetClassData() const { return &_class_data_; }


void NavigationPath::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message& to_msg, const ::PROTOBUF_NAMESPACE_ID::Message& from_msg) {
  auto* const _this = static_cast<NavigationPath*>(&to_msg);
  auto& from = static_cast<const NavigationPath&>(from_msg);
  // @@protoc_insertion_point(class_specific_merge_from_start:autonomous_proto.NavigationPath)
  GOOGLE_DCHECK_NE(&from, _this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  _this->_impl_.position_.MergeFrom(from._impl_.position_);
  if (from._internal_has_header()) {
    _this->_internal_mutable_header()->::autonomous_proto::MessageInfo::MergeFrom(
        from._internal_header());
  }
  _this->_internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void NavigationPath::CopyFrom(const NavigationPath& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:autonomous_proto.NavigationPath)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool NavigationPath::IsInitialized() const {
  return true;
}

void NavigationPath::InternalSwap(NavigationPath* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  _impl_.position_.InternalSwap(&other->_impl_.position_);
  swap(_impl_.header_, other->_impl_.header_);
}

::PROTOBUF_NAMESPACE_ID::Metadata NavigationPath::GetMetadata() const {
  return ::_pbi::AssignDescriptors(
      &descriptor_table_navigation_5fpath_2eproto_getter, &descriptor_table_navigation_5fpath_2eproto_once,
      file_level_metadata_navigation_5fpath_2eproto[0]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace autonomous_proto
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::autonomous_proto::NavigationPath*
Arena::CreateMaybeMessage< ::autonomous_proto::NavigationPath >(Arena* arena) {
  return Arena::CreateMessageInternal< ::autonomous_proto::NavigationPath >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
