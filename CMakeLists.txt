cmake_minimum_required(VERSION 3.22)

# 启用 printf 的浮点支持（必须加这行）
add_link_options(-u _printf_float)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME COD_H7_Template_Clion)

# Include toolchain file
include("cmake/gcc-arm-none-eabi.cmake")

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# ================= 核心优化点 1：全局编译标志前置 =================
# 必须放在 add_subdirectory 和 project 之前！
# 确保 FPU 硬件浮点和内核架构参数能作用于 stm32cubemx 生成的所有底层文件
set(GLOBAL_MCU_FLAGS "-mfloat-abi=hard -mfpu=fpv5-d16 -mthumb -mthumb-interwork -march=armv7e-m -mtune=cortex-m7")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GLOBAL_MCU_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GLOBAL_MCU_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} ${GLOBAL_MCU_FLAGS}")
# ===================================================================

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        DSP
)

# 在这里添加.c目录
file(GLOB_RECURSE SOURCES
        "USER/BSP/Src/*.c"
        "USER/Components/Device/Src/*.c"
        "USER/Components/Algorithm/Src/*.c"
        "USER/Components/Controller/Src/*.c"
        "USER/Application/Task/Src/*.c"
        "USER/Application/Task/Arm/*.c"
        # 可按需添加其他目录
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${SOURCES}
)

# Add include paths   在这里添加.h目录
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        USER/Components/Device/Inc
        USER/Components/Algorithm/Inc
        USER/Components/Controller/Inc
        USER/Application/Task/Inc
        USER/Application/Task/Arm
        USER/BSP/Inc
        Middlewares/ST/STM32_USB_Device_Library/Core/Inc
)

# ================= 核心优化点 2：补全必需的宏定义 =================
# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        ARM_MATH_CM7        # 启用 Cortex-M7 内核专属 DSP 指令加速
        __FPU_PRESENT=1     # 声明存在硬件 FPU，触发 system_stm32h7xx.c 开启 CPACR 寄存器
)
# ==================================================================

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        arm_cortexM7lfdp_math
)

add_custom_command(
        TARGET ${CMAKE_PROJECT_NAME}
        POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} ${CMAKE_PROJECT_NAME}.elf -Oihex ${CMAKE_PROJECT_NAME}.hex
        COMMENT "Generating hex file for STM32"
)