find_package(Eigen3 REQUIRED)

file(GLOB SRCS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")
foreach (SRC ${SRCS})
    add_executable(example_${SRC}
            ${SRC})
    target_link_libraries(example_${SRC} PUBLIC
            rbpodo
            Eigen3::Eigen)
endforeach ()