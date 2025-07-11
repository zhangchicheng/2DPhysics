# Top-level CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

project(My2DEngineProject VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find top-level dependencies required by the whole project
find_package(SDL2 REQUIRED)

# Add our subdirectories to the build
add_subdirectory(vendor/imgui)
add_subdirectory(engine)
add_subdirectory(app)
