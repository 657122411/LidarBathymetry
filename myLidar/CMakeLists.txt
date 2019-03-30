project( myLidar )
cmake_minimum_required( VERSION 3.5 )

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )

include_directories(/Users/taojianhao/CLionProjects/myLidar/myLidar/levmar)

set(SOURCES
        /Users/taojianhao/CLionProjects/myLidar/myLidar/DeepWave.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/DeepWave.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/HS_Lidar.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/HS_Lidar.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/HS_Lidar_Channel.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/HS_Lidar_Header.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/ReadFile.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/myLidar.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/ReadFile.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/TimeConvert.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/TimeConvert.h
        /Users/taojianhao/CLionProjects/myLidar/myLidar/WaveData.cpp
        /Users/taojianhao/CLionProjects/myLidar/myLidar/WaveData.h

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