#include "rrt.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include <set>
#include <cmath>
#include <random>
#include <ctime>

#include <iostream>

RRT::RRT (sf::RenderWindow& stateSpace, float growthFactor, std::vector<float> start, std::vector<float> end, float tolerance)
    : _stateSpace(stateSpace) {
    
    std::vector<sf::RectangleShape> _obstacles = {};
    _growthFactor = growthFactor;
    _tolerance = tolerance;

    _startPosition = start;
    _endPosition = end;

    std::set<Node*> children;
    _startNode = Node(_startPosition, nullptr, children);

    _nodes.push_back(_startNode);

    _mt = std::mt19937(time(nullptr));
};

void RRT::normalizeNodeToGrowthFactor(Node closestNode, Node& newNode){
    float distance = getEuclideanDistance(closestNode.getPosition(), newNode.getPosition());
    
    if (distance > _growthFactor){
        float delta_x = newNode.getPosition().at(0) - closestNode.getPosition().at(0);
        float delta_y = newNode.getPosition().at(1) - closestNode.getPosition().at(1);
        
        float normalized_x = (delta_x / distance) * _growthFactor;
        float normalized_y = (delta_y / distance) * _growthFactor;

        float new_x = normalized_x + closestNode.getPosition().at(0);
        float new_y = normalized_y + closestNode.getPosition().at(1);

        newNode.setPosition({new_x, new_y});
    }
    return;
}

float RRT::getEuclideanDistance(std::vector<float> node1Position, std::vector<float> node2Position){
    float delta_x = node1Position.at(0) - node2Position.at(0);
    float delta_y = node1Position.at(1) - node2Position.at(1);

    float distance = std::sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    return distance;
}

void RRT::getClosestNode(Node newNode, Node& closestNode){
    closestNode = _nodes.at(0);
    float distanceToClosest = getEuclideanDistance(closestNode.getPosition(), newNode.getPosition());
    for (int i = 0; i < _nodes.size(); i++){
        Node currentNode = _nodes.at(i);
        float currentDistance = getEuclideanDistance(currentNode.getPosition(), newNode.getPosition());

        if (currentDistance < distanceToClosest){
            distanceToClosest = currentDistance;
            closestNode = currentNode;
        }
    }
    return;
}

bool RRT::isCollision(std::vector<float>){
    return false;
}

void RRT::addObstacle(sf::RectangleShape obstacle){
    _obstacles.push_back(obstacle);
}

void RRT::update(){
    // steps: generate new point
    std::uniform_real_distribution<float> x_dist(0, _stateSpace.getSize().x);
    std::uniform_real_distribution<float> y_dist(0, _stateSpace.getSize().y);

    std::vector<float> newPoint = {x_dist(_mt), y_dist(_mt)};

    std::set<Node*> children;
    Node newNode(newPoint, nullptr, children);

    // find closest point
    Node closestNode;
    getClosestNode(newNode, closestNode);

    // normalize it to length of growth factor
    normalizeNodeToGrowthFactor(closestNode, newNode);

    // check for collision with obstacles between closest point and current point
        // if collision -> toss out point, start again
        // no collision -> add to tree, continue
    
    newNode.setParent(&closestNode);
    _nodes.push_back(newNode);

    // is it within range of end goal? 
        // yes -> done, back trace parents to start node 
        // no -> continue
}