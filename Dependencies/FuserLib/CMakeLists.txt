# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

set(SOURCES
    "Include/FuserLib.h"
    "Include/LegacySerialization.h"
    "Include/pch.h"
    "Include/SensorFilter.h"
    "Include/SensorSample.h"
    "Include/SensorSampleQueue.h"
    "Include/SensorSampleSerializable.h"
    "Include/TimeUnits.h"
    "Source/LegacySerialization.cpp"
    "Source/pch.cpp"
    "Source/SensorSample.cpp"
    "Source/SensorSampleQueue.cpp")

add_library(FuserLib ${SOURCES})

target_include_directories(FuserLib
    PRIVATE "Source"
    PUBLIC "Include")

# TODO: This is a circular dependency required (at present) to get
# to the "Device/IMUCharacterization.h" file and its dependencies.
# To remove this circuilar dependency, either those files need to be
# factored out into a separate dependency, or FuserLib in its entirety
# needs to be moved inside the core MAGESLAM project.
target_include_directories(FuserLib
    PRIVATE "${CMAKE_SOURCE_DIR}/Core/MAGESLAM/Source")

target_link_libraries(FuserLib
    PUBLIC cereal
    PUBLIC Arcana)

set_property(TARGET FuserLib PROPERTY FOLDER Dependencies)
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
