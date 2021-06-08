cmake_minimum_required(VERSION 3.5)
project(marine_presenter)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "srv/ButtonPress.srv")


if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

install(DIRECTORY templates objects launch
  DESTINATION share/${PROJECT_NAME})

FILE(GLOB SCRIPTS scripts/*.py)
install(PROGRAMS
  ${SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)
  
ament_package()
