cmake_minimum_required(VERSION 3.0.2)
project(trajectory_utils)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Save.srv
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include(FetchContent)
FetchContent_Declare(
  mcap_builder
  GIT_REPOSITORY https://github.com/olympus-robotics/mcap_builder.git
  GIT_TAG main
)
FetchContent_MakeAvailable(mcap_builder)


include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(trajectory_reader src/reader.cpp)
target_link_libraries(trajectory_reader ${catkin_LIBRARIES} mcap nlohmann_json::nlohmann_json)
add_dependencies(trajectory_reader trajectory_utils_generate_messages_cpp)

add_executable(trajectory_saver src/saver.cpp)
target_link_libraries(trajectory_saver ${catkin_LIBRARIES} mcap nlohmann_json::nlohmann_json)
add_dependencies(trajectory_saver trajectory_utils_generate_messages_cpp)
