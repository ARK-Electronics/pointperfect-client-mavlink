cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointperfect-client-mavlink VERSION 0.9 LANGUAGES CXX)

add_compile_options(-Wall -Wunused)

# TLS for the NTRIP connection to the PointPerfect caster
find_package(OpenSSL REQUIRED)

# Assumes MAVSDK system wide install
list(APPEND CMAKE_PREFIX_PATH "/usr/local/MAVSDK/install")
find_package(MAVSDK REQUIRED)

include_directories(third_party/tomlplusplus/)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/PointPerfectClientMavlink.cpp
    src/UbxFrameScanner.cpp)

target_link_libraries(${PROJECT_NAME}
    pthread
    OpenSSL::SSL
    OpenSSL::Crypto
    MAVSDK::mavsdk)
