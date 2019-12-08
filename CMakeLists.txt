project(LidarBathymetry)
cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

#包含LM解算需要的levmar库的所在路径
include_directories(/Users/taojianhao/CLionProjects/LidarBathymetry/levmar)

set(SOURCES
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/DeepWave.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/DeepWave.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/HS_Lidar.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/HS_Lidar.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/HS_Lidar_Channel.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/HS_Lidar_Header.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/ReadFile.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/myLidar.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/ReadFile.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/TimeConvert.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/TimeConvert.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/cpp/WaveData.cpp
        /Users/taojianhao/CLionProjects/LidarBathymetry/header/WaveData.h

        #Axb.c compiler.h levmar.h levmar.c lm.c lm.h misc.c misc.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/Axb.c
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/compiler.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/levmar.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/lm.c
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/lm.h
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/misc.c
        /Users/taojianhao/CLionProjects/LidarBathymetry/levmar/misc.h
        )

add_executable(LidarBathymetry ${SOURCES})