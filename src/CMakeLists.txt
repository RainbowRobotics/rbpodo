add_library(rbpodo
        socket.cpp
        cobot.cpp
        cobot_data.cpp)
target_include_directories(rbpodo PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
if(WIN32)
    target_link_libraries(rbpodo PUBLIC wsock32 ws2_32)
endif()
set_property(TARGET rbpodo PROPERTY POSITION_INDEPENDENT_CODE ON)

add_library(rbpodo::rbpodo ALIAS rbpodo)