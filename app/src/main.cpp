#include <iostream>
#include <vector>
#include <memory>
#include <SDL.h>
#include <cstdlib>
#include <ctime>

// Engine headers
#include <Vec2.h>
#include <Body.h>
#include <CircleShape.h>
#include <PolygonShape.h>
#include <Collision.h>

// ImGui headers
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

// --- Constants ---
const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;
const float TIME_PER_FRAME = 1.0f / 60.0f;

// --- Main Application ---
int main(int argc, char *argv[])
{
    // --- Setup (SDL, ImGui) ---
    SDL_Init(SDL_INIT_VIDEO);
    srand(time(NULL));
    SDL_Window *window = SDL_CreateWindow("Polygon Collision (SAT Demo)", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);
    ImGui::StyleColorsDark();

    // --- Game Setup ---
    bool isRunning = true;
    std::vector<std::unique_ptr<Body>> bodies;

    const float floorWidth = WINDOW_WIDTH;
    const float floorHeight = 30.0f;
    std::vector<Vec2> floorVertices = {
        {-floorWidth / 2.0f, -floorHeight / 2.0f},
        {floorWidth / 2.0f, -floorHeight / 2.0f},
        {floorWidth / 2.0f, floorHeight / 2.0f},
        {-floorWidth / 2.0f, floorHeight / 2.0f}};
    bodies.push_back(std::make_unique<Body>(PolygonShape(floorVertices, 0.0f), WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT - (floorHeight / 2.0f)));

    // --- Main Loop ---
    float spawnTimer = 0.0f;
    while (isRunning)
    {
        // --- Event Handling ---
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                isRunning = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE)
                isRunning = false;
        }

        // --- Spawning New Bodies ---
        spawnTimer += TIME_PER_FRAME;
        if (spawnTimer > 0.5f)
        {
            if (bodies.size() < 20)
            {
                std::vector<Vec2> boxVertices = {{-30, -30}, {30, -30}, {30, 30}, {-30, 30}};
                bodies.push_back(std::make_unique<Body>(
                    PolygonShape(boxVertices, 5.0f),
                    100 + rand() % (WINDOW_WIDTH - 200),
                    50 + rand() % 150));
            }
            spawnTimer = 0.0f;
        }

        // --- Physics Update ---
        for (auto &body : bodies)
        {
            if (body->inverseMass != 0.0f)
            {
                body->AddForce(Vec2(0.0, 980.0f * body->shape->mass));
            }
        }
        for (auto &body : bodies)
        {
            body->Integrate(TIME_PER_FRAME);
        }

        // --- Collision Detection and Resolution ---
        Collision::DetectAndResolveCollisions(bodies);

        // --- Rendering ---
        SDL_SetRenderDrawColor(renderer, 10, 10, 30, 255);
        SDL_RenderClear(renderer);

        for (const auto &body : bodies)
        {
            if (body->inverseMass == 0.0f)
            {
                SDL_SetRenderDrawColor(renderer, 0, 255, 100, 255);
            }
            else
            {
                SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            }

            if (body->shape->GetType() == Shape::POLYGON)
            {
                PolygonShape *poly = static_cast<PolygonShape *>(body->shape.get());
                std::vector<SDL_Point> sdlPoints;
                for (const auto &v : poly->worldVertices)
                {
                    sdlPoints.push_back({(int)v.x, (int)v.y});
                }
                if (sdlPoints.size() > 0)
                {
                    // --- THIS IS THE CORRECTED LINE ---
                    // Access the first vertex using the array subscript operator [0]
                    sdlPoints.push_back({(int)poly->worldVertices[0].x, (int)poly->worldVertices[0].y});
                    SDL_RenderDrawLines(renderer, sdlPoints.data(), sdlPoints.size());
                }
            }
        }

        // --- ImGui ---
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Info");
        ImGui::Text("Boxes are spawned periodically.");
        ImGui::Text("Collision is detected using the Separating Axis Theorem (SAT).");
        ImGui::Text("Body count: %zu", bodies.size());
        ImGui::End();
        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);

        SDL_RenderPresent(renderer);
    }

    // --- Cleanup ---
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}