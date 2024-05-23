cmake_minimum_required(VERSION 3.10)
project(autonomous_proto)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf 3 REQUIRED)
if(Protobuf_FOUND)
    message(STATUS "Protobuf found!")
else()
    message(FATAL_ERROR "Protobuf not found!")
endif()
#set(Protobuf_PROTOC_EXECUTABLE "/usr/local/bin/protoc")
message(STATUS "Using Protobuf Compiler: ${Protobuf_PROTOC_EXECUTABLE}")
message(STATUS "Using Protobuf Version: ${Protobuf_VERSION}")

set(PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/proto)
file(GLOB_RECURSE PROTO_FILES ${PROTO_DIR}/*.proto)
message(STATUS "PROTO_FILES: ${PROTO_FILES}")

set(CPP_OUT_DIR ${PROTO_DIR}/cpp_out)
file(MAKE_DIRECTORY ${CPP_OUT_DIR})
foreach(proto_file IN LISTS PROTO_FILES)
    EXECUTE_PROCESS(COMMAND ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${PROTO_DIR} --cpp_out=${CPP_OUT_DIR} ${proto_file})
endforeach()

set(PYTHON_OUT_DIR ${PROTO_DIR}/python_out)
file(MAKE_DIRECTORY ${PYTHON_OUT_DIR})
foreach(proto_file IN LISTS PROTO_FILES)
    EXECUTE_PROCESS(COMMAND ${Protobuf_PROTOC_EXECUTABLE} --proto_path=${PROTO_DIR} --python_out=${PYTHON_OUT_DIR} ${proto_file})
endforeach()

#if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
#    file(GLOB_RECURSE PROTO_CPP_SRC ${CPP_OUT_DIR}/*.pb.cc)
#    add_library(autonomous_proto_static STATIC
#            ${PROTO_CPP_SRC}
#    )
#    set(TARGET autonomous_proto_static)
#    target_compile_options(${TARGET} PRIVATE -O3)
#    target_compile_options(${TARGET} PRIVATE -g0)
#    target_compile_options(${TARGET} PRIVATE -DNDEBUG)
#    target_compile_options(${TARGET} PRIVATE -ffunction-sections -fdata-sections)
#    target_link_libraries(${TARGET} PRIVATE "-Wl,--gc-sections")
#    target_link_libraries(${TARGET} PRIVATE "-Wl,--strip-all")
#    target_link_libraries(${TARGET} PRIVATE "-flto")
#    target_include_directories(autonomous_proto_static PUBLIC
#            ${CMAKE_CURRENT_SOURCE_DIR}/include
#            ${CPP_OUT_DIR}
#            ${Protobuf_INCLUDE_DIRS}
#    )
#    target_link_libraries(autonomous_proto_static PUBLIC
#            /usr/local/lib/libprotobuf.a
#    )
#    install(TARGETS autonomous_proto_static DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}/lib)
#endif ()

add_library(autonomous_proto INTERFACE)

file(GLOB_RECURSE PROTO_CPP_SRC ${CPP_OUT_DIR}/*.pb.cc)
target_sources(autonomous_proto INTERFACE ${PROTO_CPP_SRC})

target_include_directories(autonomous_proto INTERFACE
        include
        ${CPP_OUT_DIR}
        ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(autonomous_proto INTERFACE
#        ${CMAKE_CURRENT_SOURCE_DIR}/lib/libautonomous_proto_static.a
        /usr/local/lib/libprotobuf.a
#        ${Protobuf_LIBRARIES}
)
