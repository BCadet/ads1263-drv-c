cmake_minimum_required(VERSION 3.16)

project(ads1263-drv-c)

add_library(ads1263-drv-c ${CMAKE_CURRENT_LIST_DIR}/ads1263.c)

target_include_directories(ads1263-drv-c INTERFACE ${CMAKE_CURRENT_LIST_DIR})

option(ADS1263_DOCUMENTATION "build ads1263-drv-c documentation with Doxygen" OFF)
if(ADS1263_DOCUMENTATION)
    find_package(Doxygen REQUIRED dot)
    set(DOXYGEN_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/documentation/${PROJECT_NAME}")
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE "README.md")
    set(DOXYGEN_PROJECT_NAME "${PROJECT_NAME}")
    set(DOXYGEN_GENERATE_TAGFILE "${CMAKE_BINARY_DIR}/documentation/${PROJECT_NAME}.tag")
    set(DOXYGEN_GENERATE_TREEVIEW "YES")
    doxygen_add_docs(${PROJECT_NAME}-doc .)
endif()