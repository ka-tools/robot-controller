cmake_minimum_required(VERSION 3.10)
project(RobotController)


set(CMAKE_CXX_STANDARD 17)


include_directories(include orocos-library/Headers orocos-library/Headers/utilities orocos-library/eigen-3.4.0 yamlcpp-static-library/include)

file(GLOB SOURCES 
    "orocos-library/Resources/*"
    "src/*.cpp"
    )

add_executable(robot_controller ${SOURCES})

if (WIN32)
        target_link_libraries(robot_controller
         ${CMAKE_SOURCE_DIR}/yamlcpp-static-library/lib/libyaml-cpp-windows.a
        )
endif (WIN32)

if (UNIX)
        target_link_libraries(robot_controller
                ${CMAKE_SOURCE_DIR}/yamlcpp-static-library/lib/libyaml-cpp-linux.a
        )       
endif(UNIX)

target_compile_definitions(robot_controller PRIVATE YAML_CPP_STATIC_DEFINE)
