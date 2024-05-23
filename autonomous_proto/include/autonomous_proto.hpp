#ifndef AUTONOMOUS_PROTO_AUTONOMOUS_PROTO_HPP
#define AUTONOMOUS_PROTO_AUTONOMOUS_PROTO_HPP

#include <variant>
#include "control.pb.h"
#include "local_path.pb.h"
#include "message_info.pb.h"
#include "navigation.pb.h"
#include "vehicle_state.pb.h"
#include "communication_command.pb.h"
#include "navigation_path.pb.h"
#include "sphere.pb.h"

namespace autonomous_proto {

#define STRINGFY(x) \
    case x: \
        return #x

inline std::string vehicle_state_gear_string(const VehicleState_Gear_Mode &gear_mode) {
    switch (gear_mode) {
        STRINGFY(VehicleState_Gear_Mode_unknown);
        STRINGFY(VehicleState_Gear_Mode_N);
        STRINGFY(VehicleState_Gear_Mode_D);
        STRINGFY(VehicleState_Gear_Mode_R);
        STRINGFY(VehicleState_Gear_Mode_P);
        STRINGFY(VehicleState_Gear_Mode_spin);
        STRINGFY(VehicleState_Gear_Mode_r_spin);
        STRINGFY(VehicleState_Gear_Mode_halt);
        STRINGFY(VehicleState_Gear_Mode_undefined);
        STRINGFY(VehicleState_Gear_Mode_VehicleState_Gear_Mode_INT_MIN_SENTINEL_DO_NOT_USE_);
        STRINGFY(VehicleState_Gear_Mode_VehicleState_Gear_Mode_INT_MAX_SENTINEL_DO_NOT_USE_);
    }
    return "";
}

#undef STRINGFY

struct PlainHeader {
    union {
        uint64_t value{0};
        uint8_t level[8];
    };
    std::string name{};
};

struct Plain {
    PlainHeader header{};
    std::variant<std::vector<int64_t>, std::vector<double>, std::vector<std::string>> data{};

    Plain(PlainHeader in_header, std::vector<int64_t>&& in_data) : header(std::move(in_header)), data(std::move(in_data)) {}
    Plain(PlainHeader in_header, std::vector<double>&& in_data) : header(std::move(in_header)), data(std::move(in_data)) {}
    Plain(PlainHeader in_header, std::vector<std::string>&& in_data) : header(std::move(in_header)), data(std::move(in_data)) {}
};

template<typename T, typename Get, typename RepeatedGet>
void ProcessField(const google::protobuf::Message &msg, const google::protobuf::Reflection *r, const google::protobuf::FieldDescriptor *f, Get get, RepeatedGet repeated_get, std::vector<Plain> &in_plains, const PlainHeader &in_header) {
    static std::vector<T> data;
    data.clear();
    if (f->is_repeated()) {
        for (int i = 0; i < r->FieldSize(msg, f); ++i) {
            data.emplace_back((r->*repeated_get)(msg, f, i));
        }
    } else {
        data.emplace_back((r->*get)(msg, f));
    }
    if (data.empty()) { return; }
    auto it = std::find_if(in_plains.begin(), in_plains.end(), [&](const Plain &in_plain) {
        return in_plain.header.value == in_header.value;
    });
    if (it == in_plains.end()) {
        in_plains.emplace_back(in_header, std::move(data));
    } else {
        auto &vec = std::get<std::vector<T>>(it->data);
        vec.insert(vec.end(), data.begin(), data.end());
    }
}

inline void ParseMessage(std::vector<Plain> &plains, const google::protobuf::Message &msg, const uint8_t in_level = 0, PlainHeader in_header = PlainHeader{}, bool check = true) { //NOLINT(misc-no-recursion)
    using namespace google::protobuf;
    const auto* r = msg.GetReflection();
    const auto* d = msg.GetDescriptor();
    if (in_level == 0) {
        in_header.name = d->name();
    }
    for (int i = 0; i < d->field_count(); ++i) {
        const auto* f = d->field(i);
        if (check && not f->is_repeated() && (f->cpp_type() == FieldDescriptor::CPPTYPE_MESSAGE || f->cpp_type() == FieldDescriptor::CPPTYPE_ENUM) && not r->HasField(msg, f)) { continue; }
        auto header = in_header;
        if (f->name() != "value") {
            header.name += "_" + f->name();
        }
        header.level[in_level] = f->number();
        switch (f->cpp_type()) {
            case FieldDescriptor::CPPTYPE_MESSAGE: {
                if (f->is_repeated()) {
                    for (int j = 0; j < r->FieldSize(msg, f); ++j) {
                        ParseMessage(plains, r->GetRepeatedMessage(msg, f, j), in_level + 1, header, false);
                    }
                    break;
                }
                ParseMessage(plains, r->GetMessage(msg, f), in_level + 1, header, check);
                break;
            }
            case FieldDescriptor::CPPTYPE_INT32: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetInt32, &Reflection::GetRepeatedInt32, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_INT64: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetInt64, &Reflection::GetRepeatedInt64, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_UINT32: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetUInt32, &Reflection::GetRepeatedUInt32, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_UINT64: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetUInt64, &Reflection::GetRepeatedUInt64, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_DOUBLE: {
                ProcessField<double>(msg, r, f, &Reflection::GetDouble, &Reflection::GetRepeatedDouble, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_FLOAT: {
                ProcessField<double>(msg, r, f, &Reflection::GetFloat, &Reflection::GetRepeatedFloat, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_BOOL: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetBool, &Reflection::GetRepeatedBool, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_ENUM: {
                ProcessField<int64_t>(msg, r, f, &Reflection::GetEnumValue, &Reflection::GetRepeatedEnumValue, plains, header);
                break;
            }
            case FieldDescriptor::CPPTYPE_STRING: {
                ProcessField<std::string>(msg, r, f, &Reflection::GetString, &Reflection::GetRepeatedString, plains, header);
                break;
            }
        }
    }
}

namespace sphere {

template <typename T>
inline auto Raw(const std::string &name, const T &data) {
    autonomous_proto::Sphere::Raw raw{};
    if constexpr (std::disjunction_v<std::is_same<T, bool>,
                                     std::is_same<T, int32_t>,
                                     std::is_same<T, uint32_t>,
                                     std::is_same<T, int64_t>,
                                     std::is_same<T, uint64_t>>) {
        raw.mutable_name()->set_value(name);
        raw.mutable_int64_value()->set_value(data);
    }
    if constexpr (std::disjunction_v<std::is_same<T, float>,
                                     std::is_same<T, double>>) {
        raw.mutable_name()->set_value(name);
        raw.mutable_float64_value()->set_value(data);
    }
    if constexpr (std::disjunction_v<std::is_same<T, std::vector<int32_t>>,
                                     std::is_same<T, std::vector<uint32_t>>,
                                     std::is_same<T, std::vector<int64_t>>,
                                     std::is_same<T, std::vector<uint64_t>>>) {
        raw.mutable_name()->set_value(name);
        raw.mutable_int64_array()->Add(data.begin(), data.end());
    }
    if constexpr (std::disjunction_v<std::is_same<T, std::vector<float>>,
                                     std::is_same<T, std::vector<double>>>) {
        raw.mutable_name()->set_value(name);
        raw.mutable_float64_array()->Add(data.begin(), data.end());
    }
    return raw;
}

inline auto Add(autonomous_proto::Sphere &sphere, autonomous_proto::Sphere::Raw &&raw) {
    sphere.mutable_data()->Add(std::move(raw));
}

}

namespace magic_data {
    inline auto get_data() {
        // it should be Navigation message, but it can also parse to VehicleState.
        return std::vector<uint8_t>{10, 60, 10, 34, 8, 255, 255, 255, 255, 255, 255, 255, 255, 255, 1, 16, 255, 255, 255, 255, 255, 255, 255, 255, 255, 1, 42, 10, 8, 157, 131, 243, 131, 130, 192, 227, 204, 23, 16, 128, 4, 24, 128, 16, 34, 4, 8, 190, 215, 2, 42, 10, 8, 210, 203, 243, 131, 130, 192, 227, 204, 23, 18, 33, 18, 9, 9, 27, 215, 221, 112, 141, 235, 66, 64, 26, 9, 9, 86, 50, 137, 252, 42, 153, 92, 64, 34, 9, 9, 90, 126, 69, 2, 45, 191, 98, 64, 26, 33, 18, 9, 9, 97, 169, 31, 89, 89, 171, 156, 191, 26, 9, 9, 110, 105, 109, 58, 137, 49, 152, 191, 34, 9, 9, 244, 197, 41, 75, 251, 25, 8, 64, 34, 33, 42, 9, 9, 206, 228, 175, 196, 181, 203, 20, 64, 50, 9, 9, 0, 181, 64, 124, 109, 59, 168, 63, 58, 9, 9, 143, 119, 132, 115, 62, 144, 210, 63, 42, 24, 42, 9, 9, 178, 156, 132, 63, 255, 136, 189, 191, 50, 9, 9, 40, 89, 7, 237, 99, 103, 176, 63, 58, 0, 50, 24, 42, 0, 50, 9, 9, 234, 48, 117, 152, 58, 76, 205, 191, 58, 9, 9, 143, 124, 71, 190, 35, 223, 33, 64};
    }
}

}

#endif //AUTONOMOUS_PROTO_AUTONOMOUS_PROTO_HPP
