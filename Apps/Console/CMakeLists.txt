add_executable(Console "console.cpp")
target_link_libraries(Console
    PRIVATE MAGESLAM)

set_property(DIRECTORY ${CMAKE_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT Console)
