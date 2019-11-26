project( myLidar )
cmake_minimum_required( VERSION 3.5 )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )

include_directories(/Users/taojianhao/CLionProjects/myLidar/myLidar/levmar)

set(SOURCES
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/DeepWave.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/DeepWave.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/HS_Lidar.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/HS_Lidar.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/HS_Lidar_Channel.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/HS_Lidar_Header.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/ReadFile.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/myLidar.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/ReadFile.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/TimeConvert.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/TimeConvert.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/cpp/WaveData.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/header/WaveData.h

        #Axb.c compiler.h levmar.h levmar.c lm.c lm.h misc.c misc.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/Axb.c
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/compiler.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/levmar.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/lm.c
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/lm.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/misc.c
        /Users/taojianhao/CLionProjects/myLidar/myLidar/levmar/misc.h
        )

add_executable(myLidar ${SOURCES})