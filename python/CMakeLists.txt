if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.27)
    cmake_policy(SET CMP0148 OLD)
endif ()

if (${CMAKE_VERSION} VERSION_GREATER_EQUAL 3.24)
    cmake_policy(SET CMP0135 OLD)
endif ()

find_package(Python 3 COMPONENTS Interpreter Development.Module REQUIRED)

include(FetchContent)
set(EXTERNALS_DIR ${CMAKE_BINARY_DIR}/external)
    message (STATUS Python_VERSION ${Python_VERSION})
    if (${Python_VERSION} VERSION_GREATER_EQUAL 3.8)
        message(STATUS "Python greater than 3.8")
        include(FetchContent)
        set(EXTERNALS_DIR ${CMAKE_BINARY_DIR}/external)
        FetchContent_Declare(libpybind11
                URL https://github.com/pybind/pybind11/archive/refs/tags/v3.0.0rc1.tar.gz
                URL_MD5 fce5b19778c8197aa6f1f53d93f405fb
                SOURCE_DIR ${EXTERNALS_DIR}/pybind11
                EXCLUDE_FROM_ALL
        )
        message(STATUS "pybind11 fetch content declare success with v3.0.0rc1")
        
        
        set(BUILD_TESTING OFF CACHE BOOL "")
        message(STATUS "fetch content make available")
        FetchContent_MakeAvailable(libpybind11)
        message(STATUS "done")
        unset(BUILD_TESTING CACHE)
    else()
        message(STATUS "Python lower than 3.8")
        include(FetchContent)
        set(EXTERNALS_DIR ${CMAKE_BINARY_DIR}/external)
        FetchContent_Declare(libpybind11
                URL https://github.com/pybind/pybind11/archive/refs/tags/v2.11.1.tar.gz
                URL_MD5 49e92f92244021912a56935918c927d0
                SOURCE_DIR ${EXTERNALS_DIR}/pybind11
                EXCLUDE_FROM_ALL
        )
        
        set(BUILD_TESTING OFF CACHE BOOL "")
        message(STATUS "fetch content make available")
        FetchContent_MakeAvailable(libpybind11)
        message(STATUS "done")
        unset(BUILD_TESTING CACHE)
    endif()

python_add_library(_rbpodo MODULE
        rbpodo.cpp
        data_type.cpp
        WITH_SOABI)
target_link_libraries(_rbpodo PRIVATE
        rbpodo
        pybind11::headers
        Eigen3::Eigen)

install(TARGETS _rbpodo DESTINATION rbpodo)
target_compile_definitions(_rbpodo PRIVATE VERSION_INFO=${PROJECT_VERSION})

#message(${Python_EXECUTABLE})
add_custom_target(generate_stub ALL
    COMMAND ${Python_EXECUTABLE} "-m" "pybind11_stubgen" "--numpy-array-use-type-var" "-o${CMAKE_CURRENT_BINARY_DIR}" "_rbpodo"
    WORKING_DIRECTORY $<TARGET_FILE_DIR:_rbpodo>
    DEPENDS _rbpodo
)
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/_rbpodo" DESTINATION rbpodo)
