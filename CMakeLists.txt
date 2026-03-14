cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico_w)
include(pico_sdk_import.cmake)
project(pico_ntp_server C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel)
pico_sdk_init()

# 強制的にマクロを定義してリンクエラーを防ぐ
add_definitions(-DconfigUSE_MUTEXES=1)
add_definitions(-DconfigUSE_RECURSIVE_MUTEXES=1)
add_definitions(-DINCLUDE_xSemaphoreGetMutexHolder=1)

add_executable(ntp_server
    src/main.c
    ${FREERTOS_KERNEL_PATH}/tasks.c
    ${FREERTOS_KERNEL_PATH}/list.c
    ${FREERTOS_KERNEL_PATH}/queue.c
    ${FREERTOS_KERNEL_PATH}/timers.c
    ${FREERTOS_KERNEL_PATH}/event_groups.c
    ${FREERTOS_KERNEL_PATH}/stream_buffer.c
    ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/port.c
    ${FREERTOS_KERNEL_PATH}/portable/MemMang/heap_4.c
)
target_include_directories(ntp_server PRIVATE src . ${FREERTOS_KERNEL_PATH}/include ${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/include)
target_link_libraries(ntp_server pico_stdlib pico_cyw43_arch_lwip_sys_freertos hardware_flash hardware_rtc)
pico_add_extra_outputs(ntp_server)
pico_enable_stdio_usb(ntp_server 1)
pico_enable_stdio_uart(ntp_server 0)
