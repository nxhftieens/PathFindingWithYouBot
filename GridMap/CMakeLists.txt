# CMakeList.txt : CMake project for GridMap, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (GridMap "src/GridMap.cpp"
                        "src/A-star.cpp"
                        "src/D-starLite.cpp"
                        "include/GridMap.hpp"
                        "include/A-star.hpp"
                        "include/D-starLite.hpp"
                        "include/Element.hpp"
                        "../Detect/include/MapFromCam.hpp"
                        "../Detect/src/MapFromCam.cpp")

add_executable (Server "src/Server.cpp" "include/Server.hpp")
add_executable (Client "src/Client.cpp")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET GridMap PROPERTY CXX_STANDARD 20)
  set_property(TARGET Server PROPERTY CXX_STANDARD 20)
  set_property(TARGET Client PROPERTY CXX_STANDARD 20)
endif()

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(GridMap PRIVATE SFML::Graphics 
                                      SFML::Network
                                      ${OpenCV_LIBS})

target_link_libraries(Server PRIVATE SFML::Network)
target_link_libraries(Client PRIVATE SFML::Network)

add_definitions(-DPROJECT_DIR="${PROJECT_SOURCE_DIR}")

# TODO: Add tests and install targets if needed.
