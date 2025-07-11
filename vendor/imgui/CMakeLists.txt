# vendor/imgui/CMakeLists.txt

# Create a static library for ImGui
add_library(ImGuiLib STATIC
    imgui.cpp
    imgui_draw.cpp
    imgui_tables.cpp
    imgui_widgets.cpp
    imgui_impl_sdl2.cpp
    imgui_impl_sdlrenderer2.cpp
)

# Tell any target that links against ImGuiLib where to find its headers
target_include_directories(ImGuiLib PUBLIC
    .
    backends
)

# --- THIS IS THE NEW, CRITICAL LINE ---
# Link ImGuiLib to SDL2 so it can find <SDL.h> and other headers.
# We use PUBLIC because the ImGui SDL backend headers expose SDL types,
# so any target including them (like our GameApp) will also need SDL.
target_link_libraries(ImGuiLib PUBLIC SDL2::SDL2)
