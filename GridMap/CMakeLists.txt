# CMakeList.txt : CMake project for GridMap, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (GridMap "src/GridMap.cpp"
                        "include/GridMap.hpp"
                        )

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET GridMap PROPERTY CXX_STANDARD 20)
endif()

include(FetchContent)
FetchContent_Declare(SFML
    GIT_REPOSITORY https://github.com/SFML/SFML.git
    GIT_TAG 3.0.0
    GIT_SHALLOW ON
    EXCLUDE_FROM_ALL
    SYSTEM)
FetchContent_MakeAvailable(SFML)

target_link_libraries(GridMap PRIVATE SFML::Graphics)


# TODO: Add tests and install targets if needed.
