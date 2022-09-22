cmake_minimum_required(VERSION 3.16)

project(ads1263)

add_library(ads1263 ${CMAKE_CURRENT_LIST_DIR}/ads1263.c)

target_include_directories(ads1263 INTERFACE ${CMAKE_CURRENT_LIST_DIR})
