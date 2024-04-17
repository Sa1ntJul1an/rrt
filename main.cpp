#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#include "node.h"
#include "rrt.h"

// mingw32-make.exe

using namespace sf;
using namespace std;

// width in height, in grid cells
const int WIDTH = 1900;
const int HEIGHT = 1000;

vector<int> STATE_SPACE = {WIDTH, HEIGHT};      // render window state space, contains obstacles, start, and end position

const int GROWTH_FACTOR = 100;                  // growth factor (euclidean distance) of new nodes being added to tree
const float TOLERANCE = 10;                     // euclidean distance tolerance to end position
const int OBSTACLE_DECETION_SEGMENTS = 10;      // number of segments between new node and closest node to check for collision

float LINE_WIDTH = 3.0;
float NODE_RADIUS = 4.0;

const vector<float> START = {HEIGHT / 2, 20};
const vector<float> END = {HEIGHT - 20, WIDTH - 10};


int main(){

    Vector2i mousePosition;

    bool rrt_running = false;
    int iteration = 0;

    int max_iterations = 1000;

    // RENDER WINDOW
    // =======================================================================
    RenderWindow renderWindow(VideoMode(WIDTH, HEIGHT), "RRT");
    renderWindow.setFramerateLimit(10);
    // =======================================================================

    RRT rrt(renderWindow, GROWTH_FACTOR, START, END, TOLERANCE, OBSTACLE_DECETION_SEGMENTS);

    // obstacle testing
    //=============================================
    //RectangleShape obstacle1;
    //obstacle1.setFillColor(Color::Green);
    //obstacle1.setSize(Vector2f(200, 100));
    //obstacle1.setPosition(Vector2f(900, 700));
    //rrt.addObstacle(obstacle1);
    //=============================================

    Font font;
    FileInputStream fontIn;
    fontIn.open("slkscr.ttf");
    font.loadFromStream(fontIn);

    Text iterationText;
    iterationText.setFillColor(Color::Red);
    iterationText.setPosition(10, 10);
    iterationText.setFont(font);
    iterationText.setCharacterSize(30);

    bool drawing_obstacle = false;
    vector<RectangleShape> obstacle_previews;
    RectangleShape obstacle_preview;
    obstacle_preview.setFillColor(Color::Green);
    Vector2i obstacle_corner;

    while(renderWindow.isOpen()){

        renderWindow.clear();

        mousePosition = Mouse::getPosition(renderWindow);

        if (drawing_obstacle){
            obstacle_preview.setSize(Vector2f(mousePosition.x - obstacle_corner.x, mousePosition.y - obstacle_corner.y));
            renderWindow.draw(obstacle_preview);
        }

        if (Mouse::isButtonPressed(Mouse::Left)){
            if (!drawing_obstacle){     // if not drawing an obstacle and mouse pressed, begin drawing it
                obstacle_corner = mousePosition;
                obstacle_preview.setPosition(Vector2f(obstacle_corner.x, obstacle_corner.y));
                drawing_obstacle = true;
            } else{                     // if drawing an obstacle and mouse clicked, freeze obstacle, add it to vector of obstacle previews and add to rrt state space
                obstacle_previews.push_back(obstacle_preview);
                rrt.addObstacle(obstacle_preview);
                drawing_obstacle = false;
            }
        } else if (Mouse::isButtonPressed(Mouse::Right)) {
            
        }

        // RRT ====================================================
        if (rrt_running){
            iteration ++;
            rrt.update();

            iterationText.setString("Iteration: " + to_string(iteration));
            renderWindow.draw(iterationText);
        }
        // =========================================================

        // KEYBOARD EVENTS =========================================
        if (Keyboard::isKeyPressed(Keyboard::Space)){   // space to pause / unpause
            rrt_running = !rrt_running;
        }
        if (Keyboard::isKeyPressed(Keyboard::R)){       // R to reset
            iteration = 0;
            rrt_running = false;
        }
        // =========================================================


        // CLOSE WINDOWS IF X PRESSED
        // ==========================================================
        Event renderWindowEvent;

        while(renderWindow.pollEvent(renderWindowEvent)){
            if(renderWindowEvent.type == Event::Closed){
                renderWindow.close();
            }
        }
        // ==========================================================

        // DRAW OBSTACLE PREVIEWS ===================================
        if (!rrt_running){
            for (int i = 0; i < obstacle_previews.size(); i++){
                renderWindow.draw(obstacle_previews.at(i));
            }
        }
        // ==========================================================

        rrt.draw();

        renderWindow.display();
    }

    return 0;
}