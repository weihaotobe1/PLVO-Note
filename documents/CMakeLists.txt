cmake_minimum_required( VERSION 2.8 )
project ( stvo_pl )

# 设置编译器g++ 编译模式为release　c++支持为c++11 编译.so文件
set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )

add_definitions(-DBOOST_NO_CXX11_SCOPED_ENUMS)

# 设置可执行文件输出位置和库文件输出位置
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3 REQUIRED )
link_directories(${OpenCV_LIBS_DIR}) #没有这一句窗口会闪退
include_directories(${OpenCV2_INCLUDE_DIRS})
# Boost C++库
find_package(Boost REQUIRED COMPONENTS regex thread system filesystem)
# YAML　　配置文件
find_package(yaml-cpp REQUIRED CONFIG PATHS ${YAML_PATHS})
# MRPT　　机器人编程工具箱
find_package(MRPT REQUIRED base opengl gui hwdrivers)
set(MRPT_DONT_USE_DBG_LIBS 1) #use release libraries for linking even if "Debug" CMake build
add_definitions(-DHAS_MRPT) #不加这一句不能出現运行界面

# 头文件（包括第三方库）MRPT可以自动添加
include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/include/
)

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ${MRPT_LIBS}
        ${PROJECT_SOURCE_DIR}/3rdparty/line_descriptor/lib/liblinedesc.so
        )
############### project ######################
# Set source files
list(APPEND SOURCEFILES
        src/auxiliar.cpp
        src/config.cpp
        src/dataset.cpp
        src/gridStructure.cpp
        src/lineIterator.cpp
        src/matching.cpp
        src/pinholeStereoCamera.cpp
        src/stereoFeatures.cpp
        src/stereoFrame.cpp
        src/stereoFrameHandler.cpp
        src/timer.cpp
        src/sceneRepresentation.cpp
        )
# 生成库文件
add_library(stvo SHARED ${SOURCEFILES})
target_link_libraries(stvo ${THIRD_PARTY_LIBS})
# 生成可执行文件
add_executable( imagesStVO app/imagesStVO.cpp )
target_link_libraries( imagesStVO stvo )



