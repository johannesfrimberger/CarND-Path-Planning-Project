project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(include_files
    src/Coordinates.h
    src/Map.h
    src/Tools.h
    src/Traffic.h
    src/Vehicle.h
    src/VehicleState.h
)

set(source_files
    src/Map.cpp
    src/Tools.cpp
    src/Traffic.cpp
    src/Vehicle.cpp
    src/VehicleState.cpp
    src/main.cpp
)

set(all_files
    ${include_files}
    ${source_files}
)

SOURCE_GROUP(include FILES ${include_files})
SOURCE_GROUP(src FILES ${source_files})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning ${all_files})

target_link_libraries(path_planning z ssl uv uWS)
