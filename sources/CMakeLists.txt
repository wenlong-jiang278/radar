cmake_minimum_required(VERSION 3.15)

add_subdirectory(ars408)

set(APP_TARGET server3_can0)
add_executable(${APP_TARGET} "${CMAKE_CURRENT_SOURCE_DIR}/server3_can0.cpp")
target_include_directories(${APP_TARGET} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories(${APP_TARGET} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}")
target_include_directories(${APP_TARGET} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/ars408")

target_link_libraries(${APP_TARGET} PRIVATE ars408)
target_link_libraries(${APP_TARGET} PRIVATE m rt dl pthread)

set(WATCHDOG system_watchdog)
add_executable(${WATCHDOG} "${CMAKE_CURRENT_SOURCE_DIR}/system_watchdog.cpp")
target_link_libraries(${WATCHDOG} PRIVATE m rt dl pthread)

install(TARGETS ${APP_TARGET} ${WATCHDOG} DESTINATION bin)
