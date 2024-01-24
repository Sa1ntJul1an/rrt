#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>

#include "rrt.h"

// mingw32-make.exe

using namespace sf;
using namespace std;

// width in height, in grid cells
const int WIDTH = 1900;
const int HEIGHT = 1000;

const int GROWTH_FACTOR = 100;
const float TOLERANCE = 10;

const vector<float> START = {HEIGHT / 2, 20};
const vector<float> END = {HEIGHT - 20, WIDTH - 10};


int main(){

    vector<float> mousePosition;

    bool rrt_running = false;
    int iteration = 0;

    int max_iterations = 1000;

    // RENDER WINDOW
    // =======================================================================
    RenderWindow renderWindow(VideoMode(WIDTH, HEIGHT), "RRT");
    renderWindow.setFramerateLimit(10);
    // =======================================================================

    RRT rrt(renderWindow, GROWTH_FACTOR, START, END, TOLERANCE);

    Font font;
    FileInputStream fontIn;
    fontIn.open("slkscr.ttf");
    font.loadFromStream(fontIn);

    Text iterationText;
    iterationText.setFillColor(Color::Red);
    iterationText.setPosition(10, 10);
    iterationText.setFont(font);
    iterationText.setCharacterSize(30);

    while(renderWindow.isOpen()){

        mousePosition = {float(Mouse::getPosition(renderWindow).x), float(Mouse::getPosition(renderWindow).y)};

        if (Mouse::isButtonPressed(Mouse::Left)){
            // do shit
        } else if (Mouse::isButtonPressed(Mouse::Right)) {
            
        }

        if (rrt_running){
            iteration ++;
            rrt.update();

        }


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


        // DRAW 
        // ==========================================================
        renderWindow.clear();

        // ==========================================================

        iterationText.setString("Iteration: " + to_string(iteration));

        renderWindow.draw(iterationText);

        renderWindow.display();

    }

    return 0;
}
