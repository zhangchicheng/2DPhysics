# game/CMakeLists.txt

# Create the final executable
add_executable(App
    src/main.cpp
)

# Link our game against the engine and ImGui libraries.
# CMake automatically handles finding the headers and dependencies
# because we set them up correctly in the other files.
target_link_libraries(App PRIVATE 
    EngineLib
    ImGuiLib
)