set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

set(CMAKE_C_COMPILER_ID GNU)
set(CMAKE_CXX_COMPILER_ID GNU)

# Locate GCC from PATH first, then from STM32Cube's managed bundle directory.
set(_STM32_GCC_HINTS)
if(DEFINED ENV{CUBE_BUNDLE_PATH})
    file(GLOB _CUBE_ENV_GCC_DIRS LIST_DIRECTORIES true
        "$ENV{CUBE_BUNDLE_PATH}/gnu-tools-for-stm32/*/bin")
    list(APPEND _STM32_GCC_HINTS ${_CUBE_ENV_GCC_DIRS})
endif()
if(CMAKE_HOST_WIN32 AND DEFINED ENV{LOCALAPPDATA})
    file(GLOB _CUBE_LOCAL_GCC_DIRS LIST_DIRECTORIES true
        "$ENV{LOCALAPPDATA}/stm32cube/bundles/gnu-tools-for-stm32/*/bin")
    list(APPEND _STM32_GCC_HINTS ${_CUBE_LOCAL_GCC_DIRS})
endif()
list(SORT _STM32_GCC_HINTS COMPARE NATURAL ORDER DESCENDING)

find_program(ARM_NONE_EABI_GCC
    NAMES arm-none-eabi-gcc arm-none-eabi-gcc.exe
    HINTS ${_STM32_GCC_HINTS})
if(NOT ARM_NONE_EABI_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc was not found in PATH or STM32Cube bundles")
endif()

get_filename_component(TOOLCHAIN_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)
if(CMAKE_HOST_WIN32)
    set(TOOLCHAIN_EXE_SUFFIX ".exe")
else()
    set(TOOLCHAIN_EXE_SUFFIX "")
endif()
set(TOOLCHAIN_PREFIX "${TOOLCHAIN_BIN_DIR}/arm-none-eabi-")

set(CMAKE_C_COMPILER                "${ARM_NONE_EABI_GCC}")
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              "${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXE_SUFFIX}")
set(CMAKE_LINKER                    "${TOOLCHAIN_PREFIX}g++${TOOLCHAIN_EXE_SUFFIX}")
set(CMAKE_OBJCOPY                   "${TOOLCHAIN_PREFIX}objcopy${TOOLCHAIN_EXE_SUFFIX}")
set(CMAKE_SIZE                      "${TOOLCHAIN_PREFIX}size${TOOLCHAIN_EXE_SUFFIX}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m3 ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32F103xx_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
