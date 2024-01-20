#include "rrt.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include <ctime>

RRT::RRT (sf::RenderWindow& stateSpace, float growthFactor, float start, float end)
    : _stateSpace(stateSpace) {
    
    std::vector<sf::RectangleShape> _obstacles = {};
    _growthFactor = growthFactor;

    _start = start;
    _end = end;

    _mt = std::mt19937(time(nullptr));
};

void RRT::addObstacle(sf::RectangleShape obstacle){
    _obstacles.push_back(obstacle);
}

void RRT::update(){
    // steps: generate new point
    // find closest point
        // normalize it to length of growth factor
            // no -> go back to 1
            // yes -> check for collision with obstacles between closest point and current point
        // is it within range of end goal? 
            // yes -> done, back trace connected nodes
            // no -> continue
}