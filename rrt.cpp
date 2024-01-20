#include "rrt.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include <set>
#include <random>
#include <ctime>

#include <iostream>

RRT::RRT (sf::RenderWindow& stateSpace, float growthFactor, std::vector<float> start, std::vector<float> end)
    : _stateSpace(stateSpace) {
    
    std::vector<sf::RectangleShape> _obstacles = {};
    _growthFactor = growthFactor;

    _startPosition = start;
    _endPosition = end;

    std::set<Node*> children;
    _startNode = Node(_startPosition, nullptr, children);

    _nodes.push_back(_startNode);

    _mt = std::mt19937(time(nullptr));
};

void RRT::addObstacle(sf::RectangleShape obstacle){
    _obstacles.push_back(obstacle);
}

void RRT::update(){
    std::uniform_real_distribution<float> x_dist(0, _stateSpace.getSize().x);
    std::uniform_real_distribution<float> y_dist(0, _stateSpace.getSize().y);

    std::vector<float> newPoint = {x_dist(_mt), y_dist(_mt)};

    // steps: generate new point
    // find closest point
        // normalize it to length of growth factor
            // no -> go back to 1
            // yes -> check for collision with obstacles between closest point and current point
        // is it within range of end goal? 
            // yes -> done, back trace connected nodes
            // no -> continue
}