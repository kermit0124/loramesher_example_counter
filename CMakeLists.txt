cmake_minimum_required(VERSION 3.16)

# include($ENV{ADF_PATH}/CMakeLists.txt)
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(counter)
