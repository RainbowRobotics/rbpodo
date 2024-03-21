file(GLOB SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
foreach (SRC ${SRCS})
    string(REGEX REPLACE "\\.[^.]*$" "" SRC_WITHOUT_EXT ${SRC})
    add_executable(example_${SRC_WITHOUT_EXT}
            ${SRC})
    target_link_libraries(example_${SRC_WITHOUT_EXT} PUBLIC
            rbpodo)
endforeach ()

if (BUILD_EIGEN_EXAMPLES)
    add_subdirectory(eigen)
endif ()