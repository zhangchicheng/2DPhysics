# engine/CMakeLists.txt

# Create a static library for our engine
add_library(EngineLib STATIC
    src/Particle.cpp
    src/Body.cpp
)

# Any project linking EngineLib needs access to its public headers
target_include_directories(EngineLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# The engine itself needs SDL, and so does any project using the engine.
# So we link SDL2 as PUBLIC.
target_link_libraries(EngineLib PUBLIC SDL2::SDL2)
