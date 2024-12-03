cmake_minimum_required(VERSION 3.15)

# 添加全局宏定义
add_definitions(-D__linux__)
add_definitions(-D__ARM_NEON)
add_definitions(-D_GNU_SOURCE)
# add_definitions(-DCONFIG_GPS)
# add_definitions(-DCONFIG_JILIN)
add_definitions(-DCONFIG_UPGRADE)
# add_definitions(-DCONFIG_CANFRAME)
add_definitions(-DCONFIG_SEARCHBRDCAST)
# add_definitions(-DCONFIG_ZZTK_PROTOCOL)
# add_definitions(-DCONFIG_SENDACTUALCLUSTER)

# 添加头文件目录
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# 添加当前目录下的源文件和头文件
file(GLOB ARS408_SOURCES "*.cpp")
file(GLOB ARS408_HEADERS "*.h")

# 设置目标名称
set(ODM_TARGET ars408)

# 添加库文件
add_library(${ODM_TARGET} ${ARS408_SOURCES} ${ARS408_HEADERS})

# 链接其他使用到的库
target_link_libraries(${ODM_TARGET} sqlite3 cJSON server)