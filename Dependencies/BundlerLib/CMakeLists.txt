# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

set(LIBRARY_NAME "BundlerLib")

set(SOURCES
    "Include/BundlerLib.h"
    "Include/PoseGraphOptimizer.h"
    "Source/BundlerLib.cpp"
    "Source/PoseGraphOptimizer.cpp")

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME} PUBLIC "Include")

target_link_libraries(${LIBRARY_NAME}
    PRIVATE g2o
    PUBLIC Eigen
    PUBLIC GSL)

target_compile_options(${LIBRARY_NAME} PRIVATE "/bigobj")

set_property(TARGET ${LIBRARY_NAME} PROPERTY FOLDER Dependencies)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
