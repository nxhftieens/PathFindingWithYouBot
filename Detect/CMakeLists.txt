# CMakeList.txt : CMake project for GridMap, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (Detect	"src/Detect.cpp"
						"include/Detect.hpp")
add_executable (MapFromCam	"src/MapFromCam.cpp"
							"include/MapFromCam.hpp")
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET Detect PROPERTY CXX_STANDARD 20)
  set_property(TARGET MapFromCam PROPERTY CXX_STANDARD 20)
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_link_libraries(Detect ${OpenCV_LIBS})
target_link_libraries(MapFromCam ${OpenCV_LIBS})


# TODO: Add tests and install targets if needed.
