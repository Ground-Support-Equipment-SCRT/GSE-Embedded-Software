cmake_minimum_required(VERSION 3.10)
project(ChessRobot)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Add source files
set(SOURCES
	main.cpp
)

# Include directories
include_directories(/usr/local/include)

# Link libraries
link_directories(/usr/local/lib)

# Create executable
add_executable(program ${SOURCES})
target_link_libraries(program ${LIBS})