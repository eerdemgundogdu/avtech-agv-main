# Arduino'dan bagimsiz firmware modulleri icin host testleri.
#
#   cmake -S firmware_tests -B build/test
#   cmake --build build/test
#   ctest --test-dir build/test --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(avtech_agv_firmware_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(FW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../teensy_firmware)
set(STRICT -Wall -Wextra -Wpedantic -Wconversion -Werror
           -fsanitize=address,undefined -fno-omit-frame-pointer)

enable_testing()

foreach(name cmd_parser hover_protocol)
    add_executable(test_${name} test_${name}.cpp ${FW_DIR}/${name}.cpp)
    target_include_directories(test_${name} PRIVATE ${FW_DIR} ${CMAKE_CURRENT_SOURCE_DIR})
    target_compile_options(test_${name} PRIVATE ${STRICT})
    target_link_options(test_${name} PRIVATE -fsanitize=address,undefined)
    add_test(NAME ${name} COMMAND test_${name})
endforeach()
