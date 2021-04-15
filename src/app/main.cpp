#include "application.h"
#include <imgui.h>
#include <iostream>
#include <math.h>
#include <deque>
#include <chrono>
#include "../boids/boids.h"

#define T float // T means float
#define dim 2 // dim means 2

class TestApp : public Application
{
#define RED      nvgRGBA(220,50,50,255) //Red
#define BLUE     nvgRGBA(50,50,220,255) //Blue
#define GREEN    nvgRGBA(50,220,50,255) //Green

typedef Matrix<T, Eigen::Dynamic, 1> VectorXT;  // VectorXT is an Eigen matrix n*1, float
typedef Matrix<T, dim, Eigen::Dynamic> TVStack; // TVStack is an Eigen matrix 2*n, float
typedef Vector<T, dim> TV;                      // TV is an Eigen vector 2*1, float

//Eigen quick ref: https://eigen.tuxfamily.org/dox/group__QuickRefPage.html

public:
    TestApp(int w, int h, const char * title) : Application(title, w, h) 
    {
        ImGui::StyleColorsClassic();
        const char* name = IMGUI_FONT_FOLDER"/Cousine-Regular.ttf";
        nvgCreateFont(vg, "sans", name);
    }

    void process() override 
    {
        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(now-lastFrame).count() >= 10./60. * 1.e6)
        {
            if(keyDown[GLFW_KEY_R])                 // if keyinput = R, initialize again (refresh)
                boids.initializePositions(currentMethod); // initialize positions and velocities according to current method
            if(keyDown[GLFW_KEY_SPACE])             // if keyinput = space, pause simulation
                boids.pause();
            if(keyDown[GLFW_KEY_ESCAPE])            // if keyinput = ESC, exit simulation
                exit(0);
            lastFrame = now;
        }
    }

    void drawImGui() override 
    {
        using namespace ImGui;
       const char* names[] = {"FreeFall", "Circular Motion", 
                            "Cohesion", "Alignment", "Separation", "Collision Avoidance",
                            "Leading","Collaborative & Adversarial"};
       Combo("Boids Behavior", (int*)&currentMethod, names, 8);
       End();
    }

    void drawNanoVG() override 
    {
        // automatically initialize when currentMethod is changed
        if(currentMethod != oldMethod)
        {
            boids.initializePositions(currentMethod);
            oldMethod = currentMethod;
        }

        boids.updateBehavior(currentMethod);
        TVStack boids_pos = boids.getPositions();
        
        // plot mapping function revised for better visulization
        // origin (0,0) is in the middle
        // scale = 0.33333, width = 720 + 360, height = 720
        // ratio---> 1:180
        auto shift_01_to_screen = [](TV pos_01, T scale, T width, T height)
        {
            return TV(0.5*(1 - scale)*width + 0.25*pos_01[0]*(1 - scale)*width, 0.5*height + 0.25*pos_01[1]*height);
        };

        // if currentMethod is collision avoidance, draw obstacles
        if (currentMethod == COLLISION_AVOID)
        {
            nvgBeginPath(vg);
            TV obs_pos = shift_01_to_screen(boids.get_obs_pos(), scale, width, height);
            nvgCircle(vg, obs_pos[0], obs_pos[1],180*boids.get_obs_radius()); // radius = 36 pixels = 0.2, type float
            nvgFillColor(vg, GREEN);
            nvgFill(vg);

            nvgBeginPath(vg);
            TV goal_pos = shift_01_to_screen(boids.get_goal_pos(), scale, width, height);
            nvgCircle(vg, goal_pos[0], goal_pos[1],5);
            nvgFillColor(vg, BLUE);
            nvgFill(vg);

        }
        
        // if currentMethod is leader, draw the leader birds
        if(currentMethod == LEADER)
        {
            TV pos = boids_pos.col(0);
            
            // drag mouse target
            nvgBeginPath(vg);
            nvgCircle(vg, mouse_pos_pixels[0], mouse_pos_pixels[1],4.f);
            nvgFillColor(vg, GREEN);
            nvgFill(vg);

            nvgBeginPath(vg);
            // just map position from 01 simulation space to screen space
            TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
            nvgCircle(vg, screen_pos[0], screen_pos[1], 4.f);
            nvgFillColor(vg, BLUE);
            nvgFill(vg);
            for(int i = 1; i < boids.getParticleNumber(); i++)
            {
                TV pos = boids_pos.col(i);
                nvgBeginPath(vg);
                // just map position from 01 simulation space to screen space
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, RED);
                nvgFill(vg);
            }
        }
        else if (currentMethod == CA_BEHAVE)
        {
            TVStack A_pos = boids.get_A_pos();
            TVStack B_pos = boids.get_B_pos();
            for(int i = 0; i < A_pos.cols(); i++)
            {
                TV pos = A_pos.col(i);
                nvgBeginPath(vg);
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, RED);
                nvgFill(vg);
            }
            for(int i = 0; i < B_pos.cols(); i++)
            {
                TV pos = B_pos.col(i);
                nvgBeginPath(vg);
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, BLUE);
                nvgFill(vg);
            }
        }
        else
        {
            // draw boids
            for(int i = 0; i < boids.getParticleNumber(); i++)
            {
                TV pos = boids_pos.col(i);
                nvgBeginPath(vg);
                // just map position from 01 simulation space to screen space
                TV screen_pos = shift_01_to_screen(TV(pos[0], pos[1]), scale, width, height);
                nvgCircle(vg, screen_pos[0], screen_pos[1], 2.f);
                nvgFillColor(vg, RED);
                nvgFill(vg);
            }
        }
    }

protected:
    void mouseButtonPressed(int button, int mods) override 
    {
        mouse_pos_pixels = TV(mouseState.lastMouseX, mouseState.lastMouseY);
        mouse_pos[0] = (mouse_pos_pixels[0] - 0.5*(1 - scale)*width)/0.25/(1 - scale)/width;
        mouse_pos[1] = (mouse_pos_pixels[1] - 0.5*height           )/0.25/height;
        boids.getMousePos(mouse_pos);
        if(currentMethod == LEADER)
        {
            std::cout<<"current leader target: ("<<mouse_pos[0]<<","<<mouse_pos[1]<<")"<<'\n';
        }
    }
    void mouseButtonReleased(int button, int mods) override {}

private:
    int loadFonts(NVGcontext* vg)
    {
        int font;
        font = nvgCreateFont(vg, "sans", "../example/Roboto-Regular.ttf");
        if (font == -1) {
            printf("Could not add font regular.\n");
            return -1;
        }
        font = nvgCreateFont(vg, "sans-bold", "../example/Roboto-Bold.ttf");
        if (font == -1) {
            printf("Could not add font bold.\n");
            return -1;
        }
        return 0;
    }

private:
    MethodTypes currentMethod = FREEFALL;
    MethodTypes oldMethod = FREEFALL;
    Boids<T, dim> boids = Boids<T, dim>(40); //<---- boids number changes here, should be even number
    std::chrono::high_resolution_clock::time_point lastFrame;
    float scale = 0.33333;
    TV mouse_pos = TV(0,0);
    TV mouse_pos_pixels = TV(0,0);
};

int main(int, char**)
{
    int width = 1080;
    int height = 720;
    TestApp app(width, height, "Assignment 3 Boids");
    app.run();
    return 0;
}
