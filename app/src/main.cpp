#include <iostream>
#include <vector>
#include <memory>
#include <SDL.h>

// Engine headers
#include <Vec2.h>
#include <Body.h>
#include <Particle.h>
#include <CircleShape.h>
#include <PolygonShape.h>

// ImGui headers
#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>

// --- Data Structures ---
struct Spring
{
    Particle *p1;
    Particle *p2;
    float restLength;
};

// --- Simulation Constants ---
const int WINDOW_WIDTH = 1280;
const int WINDOW_HEIGHT = 720;
const int TARGET_FPS = 60;
const float G = 10.0f;

// --- Global Physics Parameters ---
struct PhysicsConfig
{
    float G = 5000.0f; // Gravitational constant
    float springConstant = 50.0f;
    float dragCoefficient = 0.5f;
};

// --- Helper Functions ---
void DrawFilledCircle(SDL_Renderer *renderer, int centerX, int centerY, int radius)
{
    for (int y = -radius; y <= radius; y++)
    {
        for (int x = -radius; x <= radius; x++)
        {
            if (x * x + y * y <= radius * radius)
            {
                SDL_RenderDrawPoint(renderer, centerX + x, centerY + y);
            }
        }
    }
}

// --- Main Application ---
void KeepBodyInBounds(Body *body)
{
    const float bounceDamping = 0.7f;
    const float friction = 0.8f;

    // For this simple check, we'll treat the body's size as a fixed buffer.
    // A more advanced system would use the actual shape's extents.
    float effectiveRadius = 50.0f;
    if (body->shape->GetType() == Shape::CIRCLE)
    {
        effectiveRadius = static_cast<CircleShape *>(body->shape.get())->radius;
    }

    // Floor collision
    if (body->position.y + effectiveRadius > WINDOW_HEIGHT)
    {
        body->position.y = WINDOW_HEIGHT - effectiveRadius;
        body->velocity.y *= -bounceDamping;
        body->velocity.x *= friction;
        body->angularVelocity *= friction; // Rotational friction
    }

    // Right wall collision
    if (body->position.x + effectiveRadius > WINDOW_WIDTH)
    {
        body->position.x = WINDOW_WIDTH - effectiveRadius;
        body->velocity.x *= -bounceDamping;
        body->angularVelocity *= friction;
    }

    // Left wall collision
    if (body->position.x - effectiveRadius < 0)
    {
        body->position.x = effectiveRadius;
        body->velocity.x *= -bounceDamping;
        body->angularVelocity *= friction;
    }
}

// --- Main Application ---
int main(int argc, char *argv[])
{
    // --- Setup ---
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window *window = SDL_CreateWindow("Rigid Body Physics", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    ImGui::CreateContext();
    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);
    ImGui::StyleColorsDark();

    // --- Game Setup ---
    bool isRunning = true;
    std::vector<std::unique_ptr<Body>> bodies;

    std::vector<Vec2> boxVertices = {{-50, -50}, {50, -50}, {50, 50}, {-50, 50}};
    bodies.push_back(std::make_unique<Body>(PolygonShape(boxVertices, 5.0f), WINDOW_WIDTH / 2.0f, WINDOW_HEIGHT / 2.0f));

    bodies.push_back(std::make_unique<Body>(CircleShape(15.0f, 1.0f), 100.0f, 100.0f));

    // --- Main Loop ---
    while (isRunning)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                isRunning = false;

            // On mouse click, apply a force to the ball
            if (!ImGui::GetIO().WantCaptureMouse && event.type == SDL_MOUSEBUTTONDOWN)
            {
                int mouseX, mouseY;
                SDL_GetMouseState(&mouseX, &mouseY);
                Vec2 target(mouseX, mouseY);
                Vec2 direction = (target - bodies[1]->position).Normalized();
                bodies[1]->AddForce(direction * 5000.0f);
            }
        }

        // --- Physics Update ---
        for (auto &body : bodies)
        {
            body->AddForce(Vec2(0.0, G * body->shape->mass)); // Gravity
        }

        // Exercise: Coding Torque and Moment of Inertia
        // On right click, apply a force at an offset to the box to create torque
        if (SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(SDL_BUTTON_RIGHT))
        {
            Body *box = bodies[0].get();
            Vec2 forcePoint = box->position + Vec2(50, 50); // Apply force to top-right corner
            Vec2 force(0, -500.0f);                         // Upward force

            box->AddForce(force);

            // Torque = r x F
            Vec2 r = forcePoint - box->position;
            float torque = r.Cross(force);
            box->AddTorque(torque);
        }

        for (auto &body : bodies)
        {
            body->Integrate(1.0f / TARGET_FPS);
            KeepBodyInBounds(body.get());
        }

        // --- Rendering ---
        SDL_SetRenderDrawColor(renderer, 10, 10, 30, 255);
        SDL_RenderClear(renderer);

        for (const auto &body : bodies)
        {
            SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
            if (body->shape->GetType() == Shape::CIRCLE)
            {
                CircleShape *circle = static_cast<CircleShape *>(body->shape.get());
                DrawFilledCircle(renderer, body->position.x, body->position.y, circle->radius);
            }
            else if (body->shape->GetType() == Shape::POLYGON)
            {
                PolygonShape *poly = static_cast<PolygonShape *>(body->shape.get());
                // SDL wants an array of SDL_Points
                std::vector<SDL_Point> sdlPoints;
                for (const auto &v : poly->worldVertices)
                {
                    sdlPoints.push_back({(int)v.x, (int)v.y});
                }
                // Close the loop
                sdlPoints.push_back({(int)poly->worldVertices[0].x, (int)poly->worldVertices[0].y});
                SDL_RenderDrawLines(renderer, sdlPoints.data(), sdlPoints.size());
            }
        }

        // --- ImGui ---
        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Controls");
        ImGui::Text("Left-click to apply force to the ball.");
        ImGui::Text("Right-click to apply torque to the box.");
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