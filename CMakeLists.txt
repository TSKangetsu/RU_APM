set(MPUDriver 'pigpio' CACHE  INTERNAL '') 
set(MODULE_APM true  CACHE BOOL "enable module: singleflight APM controller")
set(MODULE_FFMPEG true  CACHE BOOL "enable module: H.264 or H.265 transcodec")
set(MODULE_FECLIB true  CACHE BOOL "enable module: Reed solomon forward error correct")
set(FULLSTATIC false CACHE  INTERNAL bool)

cmake_minimum_required(VERSION 3.13)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(THREADS_PREFER_PTHREAD_FLAG ON)

project(Ru_APM)

IF(FULLSTATIC)
  set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
  set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
  set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
  set(BUILD_SHARED_LIBS OFF)
  set(CMAKE_EXE_LINKER_FLAGS "-static -static-libstdc++ -static-libgcc")
  set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
ENDIF()

add_definitions(-DDEBUG)
add_definitions(-DSELF_SIG_MANAGE)

if(${MODULE_FFMPEG})
    add_definitions(-DMODULE_FFMPEG)
    find_path(AVCODEC_INCLUDE_DIR libavcodec/avcodec.h)
    find_library(AVCODEC_LIBRARY avcodec)
    find_path(AVFORMAT_INCLUDE_DIR libavformat/avformat.h)
    find_library(AVFORMAT_LIBRARY avformat)
    find_path(AVUTIL_INCLUDE_DIR libavutil/avutil.h)
    find_library(AVUTIL_LIBRARY avutil)
    find_path(AVDEVICE_INCLUDE_DIR libavdevice/avdevice.h)
    find_library(AVDEVICE_LIBRARY avdevice)
    find_path(AVFILTER_INCLUDE_DIR libavfilter/avfilter.h)
    find_library(AVFILTER_LIBRARY avfilter)
    find_path(SWSCALE_INCLUDE_DIR libswscale/swscale.h)
    find_library(SWSCALE_LIBRARY swscale)
    set(AV_Tool_LIB ${AVCODEC_LIBRARY} ${AVFORMAT_LIBRARY} ${AVUTIL_LIBRARY} ${AVDEVICE_LIBRARY} ${AVFILTER_LIBRARY} ${SWSCALE_LIBRARY})
    set(INC_DIRS ${AVCODEC_INCLUDE_DIR} ${AVFORMAT_INCLUDE_DIR} ${AVUTIL_INCLUDE_DIR} ${AVDEVICE_INCLUDE_DIR} ${AVFILTER_INCLUDE_DIR} ${SWSCALE_INCLUDE_DIR})
    include_directories(${INC_DIRS})

    aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/SRC/_Thirdparty/FFMPEG/ FFMPEG)
endif()

if(${MODULE_FECLIB})
    add_definitions(-DMODULE_FECLIB)
endif()

find_package(Threads REQUIRED)

if(${MODULE_APM})
    add_subdirectory(SRC/RPiSingleAPM/src)
endif()

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/SRC/_Excutable/CameraDrive/ v4l2)

add_executable(RuAPM_Cli SRC/Launch.cpp ${v4l2} "$<$<BOOL:${MODULE_FFMPEG}>:${FFMPEG}>")

target_link_libraries(RuAPM_Cli  "$<$<BOOL:${MODULE_APM}>:${RPiSingleAPI}>" )
target_link_libraries(RuAPM_Cli Threads::Threads)
target_link_libraries(RuAPM_Cli "$<$<BOOL:${MODULE_FFMPEG}>:${AV_Tool_LIB}>")
