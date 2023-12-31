cmake_minimum_required(VERSION 3.10)
project(simulated_edscorbot_runtime)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

execute_process(
    COMMAND bash "-c" "'${CMAKE_CURRENT_SOURCE_DIR}/mosquitto_build.sh'" 
    OUTPUT_VARIABLE FOO
)

add_subdirectory("json")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fPIC --static  -DEDS_VERBOSE")
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} --static -DEDS_VERBOSE")
set(LANGUAGE C_STANDARD)

#uncomment when putting all files generated by the tool
add_executable(Edscorbot_enrique src/simulated_server.cpp src/devmem.cpp)
target_include_directories(Edscorbot_enrique PUBLIC "src" "./" "json/single_include/" "mosquitto/include/")
TARGET_LINK_LIBRARIES(Edscorbot_enrique PRIVATE  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libmosquitto_static.a" -lpthread)

#add_executable(Edscorbot_SimTest src_test/simulated_server.cpp)
#target_include_directories(Edscorbot_SimTest PUBLIC "src_test" "./" "json/single_include/" "mosquitto/include/")
#TARGET_LINK_LIBRARIES(Edscorbot_SimTest PRIVATE  "${CMAKE_CURRENT_SOURCE_DIR}/lib/libmosquitto_static.a" -lpthread)


SET(CMAKE_BUILD_TYPE "Debug")