cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(pointperfect-client-mavlink VERSION 0.9 LANGUAGES CXX)

add_compile_options(-Wall -Wunused)

# TLS for the MQTT connection to the PointPerfect broker
find_package(OpenSSL REQUIRED)

# Fetch the Eclipse Paho MQTT C++ client (and build the bundled C client with it).
include(FetchContent)
set(PAHO_WITH_MQTT_C ON CACHE INTERNAL "")
set(PAHO_WITH_SSL ON CACHE INTERNAL "")
set(PAHO_BUILD_STATIC ON CACHE INTERNAL "")
set(PAHO_BUILD_SHARED OFF CACHE INTERNAL "")
set(PAHO_BUILD_SAMPLES OFF CACHE INTERNAL "")
set(PAHO_BUILD_TESTS OFF CACHE INTERNAL "")
set(PAHO_BUILD_DOCUMENTATION OFF CACHE INTERNAL "")
FetchContent_Declare(
    paho_mqtt_cpp
    GIT_REPOSITORY https://github.com/eclipse-paho/paho.mqtt.cpp.git
    GIT_TAG v1.4.1)
FetchContent_MakeAvailable(paho_mqtt_cpp)

# Assumes MAVSDK system wide install
list(APPEND CMAKE_PREFIX_PATH "/usr/local/MAVSDK/install")
find_package(MAVSDK REQUIRED)

include_directories(third_party/tomlplusplus/)

add_executable(${PROJECT_NAME}
    src/main.cpp
    src/PointPerfectClientMavlink.cpp)

target_link_libraries(${PROJECT_NAME}
    pthread
    PahoMqttCpp::paho-mqttpp3-static
    MAVSDK::mavsdk)
