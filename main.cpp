#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#include "node.h"
#include "rrt.h"


using namespace sf;
using namespace std;

// width in height, in grid cells
const int WIDTH = 1900;
const int HEIGHT = 1000;

vector<int> STATE_SPACE = {WIDTH, HEIGHT};      // render window state space, contains obstacles, start, and end position

const int GROWTH_FACTOR = 50;                  // growth factor (euclidean distance) of new nodes being added to tree
const float TOLERANCE = 40;                     // euclidean distance tolerance to end position
const int OBSTACLE_DECETION_SEGMENTS = 15;      // number of segments between new node and closest node to check for collision


const vector<float> START = {200, 300};
const vector<float> END = {1500, 900};

const float BIAS_TOWARDS_GOAL = 0.6;

int main(){

    Vector2i mousePosition;

    bool rrt_running = false;
    int iteration = 0;

    // RENDER WINDOW
    // =========================================================
    RenderWindow renderWindow(VideoMode(WIDTH, HEIGHT), "RRT");
    renderWindow.setFramerateLimit(60);
    // =========================================================

    RRT rrt(renderWindow, GROWTH_FACTOR, START, END, TOLERANCE, OBSTACLE_DECETION_SEGMENTS, BIAS_TOWARDS_GOAL);

    Font font;
    FileInputStream fontIn;
    fontIn.open("slkscr.ttf");
    font.loadFromStream(fontIn);

    Text iterationText;
    iterationText.setFillColor(Color::White);
    iterationText.setPosition(10, 10);
    iterationText.setFont(font);
    iterationText.setCharacterSize(30);

    bool drawing_obstacle = false;
    vector<RectangleShape> obstacle_previews;
    RectangleShape obstacle_preview;
    obstacle_preview.setFillColor(Color::Magenta);
    Vector2i obstacle_corner;

    bool sim_started = false;
    bool goal_reached = false;

    bool spacebar_held = false;

    CircleShape start;
    CircleShape end;
    Event renderWindowEvent;

    while(renderWindow.isOpen()){

        renderWindow.clear();

        mousePosition = Mouse::getPosition(renderWindow);

        if (drawing_obstacle){
            obstacle_preview.setSize(Vector2f(mousePosition.x - obstacle_corner.x, mousePosition.y - obstacle_corner.y));
            renderWindow.draw(obstacle_preview);
        }

        if (Mouse::isButtonPressed(Mouse::Left)){
            if (!drawing_obstacle && !rrt_running && !sim_started){     // if not drawing an obstacle, sim hasn't started, and mouse pressed, begin drawing it
                obstacle_corner = mousePosition;
                obstacle_preview.setPosition(Vector2f(obstacle_corner.x, obstacle_corner.y));
                drawing_obstacle = true;
            }
        } else if (drawing_obstacle) {      // if drawing an obstacle and mouse released, freeze obstacle, add it to vector of obstacle previews and add to rrt state space
            obstacle_previews.push_back(obstacle_preview);
            rrt.addObstacle(obstacle_preview);
            drawing_obstacle = false;
        }

        // RRT ====================================================
        if (rrt_running && !goal_reached){
            iteration ++;
            rrt.update();

            goal_reached = rrt.isGoalReached();

            iterationText.setString("Iteration: " + to_string(iteration));
        }
        // =========================================================

        // KEYBOARD EVENTS =========================================
        if (Keyboard::isKeyPressed(Keyboard::Space) && !spacebar_held){   // space to pause / unpause
            sim_started = true;
            rrt_running = !rrt_running;
            spacebar_held = true;
        } else if (!Keyboard::isKeyPressed(Keyboard::Space)) {
            spacebar_held = false;
        }
        
        // =========================================================


        // CLOSE WINDOWS IF X PRESSED
        // =========================================================

        while(renderWindow.pollEvent(renderWindowEvent)){
            if(renderWindowEvent.type == Event::Closed){
                renderWindow.close();
            }
        }
        // =========================================================

        // DRAW OBSTACLE AND START/STOP PREVIEWS ===================
        if (!rrt_running){
            for (int i = 0; i < obstacle_previews.size(); i++){
                renderWindow.draw(obstacle_previews.at(i));
            }

            start.setFillColor(Color::Green);
            start.setPosition(Vector2f(START.at(0) - TOLERANCE/2, START.at(1) - TOLERANCE/2));
            start.setRadius(TOLERANCE);
            renderWindow.draw(start);

            end.setFillColor(Color::Red);
            end.setPosition(Vector2f(END.at(0) - TOLERANCE/2, END.at(1) - TOLERANCE/2));
            end.setRadius(TOLERANCE);
            renderWindow.draw(end);
        }
        // =========================================================

        rrt.draw();
        renderWindow.draw(iterationText);

        renderWindow.display();
    }

    return 0;
}