#include "rrt.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include <set>
#include <cmath>
#include <random>
#include <ctime>

#include <iostream>

RRT::RRT (sf::RenderWindow& stateSpace, float growthFactor, std::vector<float> start, std::vector<float> end, float tolerance, int obstacle_detection_segments)
    : _stateSpace(stateSpace) {
    
    std::vector<sf::RectangleShape> _obstacles = {};
    _growthFactor = growthFactor;
    _tolerance = tolerance;
    _obstacle_detection_segments = obstacle_detection_segments;

    _startPosition = start;
    _endPosition = end;

    std::set<Node*> children;
    _startNode = Node(_startPosition, nullptr, children);

    _nodes.push_back(_startNode);

    _mt = std::mt19937(time(nullptr));
};

std::vector<float> RRT::getUnitVector(std::vector<float> point_one, std::vector<float> point_two){
    float distance = getEuclideanDistance(point_one, point_two);

    float delta_x = point_two.at(0) - point_one.at(0);
    float delta_y = point_two.at(1) - point_one.at(1);

    float unit_x = delta_x / distance;
    float unit_y = delta_y / distance;

    std::vector<float> unit_vector = {unit_x, unit_y};
    return unit_vector;
}

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
}

float RRT::getEuclideanDistance(std::vector<float> node1Position, std::vector<float> node2Position){
    float delta_x = node1Position.at(0) - node2Position.at(0);
    float delta_y = node1Position.at(1) - node2Position.at(1);

    float distance = std::sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    return distance;
}

Node RRT::sampleStateSpace(){
    // generate new point in state space
    std::uniform_real_distribution<float> x_dist(0, _stateSpace.getSize().x);
    std::uniform_real_distribution<float> y_dist(0,  _stateSpace.getSize().y);

    std::vector<float> newPoint = {x_dist(_mt), y_dist(_mt)};

    std::set<Node*> children;
    Node newNode = Node(newPoint, nullptr, children);
    return newNode;
}

Node* RRT::getClosestNode(Node newNode){
    if (_nodes.empty()){
        return nullptr;
    }

    Node* closestNode = &_nodes.at(0);
    float distanceToClosest = getEuclideanDistance(closestNode->getPosition(), newNode.getPosition());
    for (int i = 0; i < _nodes.size(); i++){
        Node* currentNode = &_nodes.at(i);
        float currentDistance = getEuclideanDistance(currentNode->getPosition(), newNode.getPosition());

        if (currentDistance < distanceToClosest){
            distanceToClosest = currentDistance;
            closestNode = currentNode;
        }
    }
    return closestNode;
}

bool RRT::isCollision(std::vector<float> point){
    for (int i = 0; i < _obstacles.size(); i++){        // for every obstacle, if the point is within the obstacles global bounds
        if (_obstacles.at(i).getGlobalBounds().contains(sf::Vector2f(point.at(0), point.at(1)))) {
            return true;                                // return true
        }
    }
    return false;                                       // return false by default
}

bool RRT::isObstacleInPath(Node parent, Node newNode){
    std::vector<float> parent_position = parent.getPosition();
    std::vector<float> newNode_position = newNode.getPosition();

    float distance_to_node = getEuclideanDistance(parent_position, newNode_position);

    if (isCollision(newNode_position)){     // if new node is in an obstacle, return true
        return true;
    }

    std::vector<float> unit_vector_parent_to_child = getUnitVector(parent_position, newNode_position);
    float segment_magnitude = distance_to_node / _obstacle_detection_segments;      // magnitude of each vector along path from parent to child node

    for (int i = 0; i < _obstacle_detection_segments; i++){
        // get x and y position of each segment along vector from parent to child
        float current_x = parent_position.at(0) + unit_vector_parent_to_child.at(0) * segment_magnitude * i;
        float current_y = parent_position.at(1) + unit_vector_parent_to_child.at(1) * segment_magnitude * i;

        std::vector<float> current_position = {current_x, current_y};

        if (isCollision(current_position)){
            return true;
        }
    }
    
    return false;
}

void RRT::addObstacle(sf::RectangleShape obstacle){
    _obstacles.push_back(obstacle);
}

void RRT::update(){
    // generate new point
    Node newNode = sampleStateSpace();

    // find closest point
    Node* closestNode = getClosestNode(newNode);

    // normalize it to length of growth factor
    normalizeNodeToGrowthFactor(*closestNode, newNode);

    // check for collision with obstacles between closest point and current point
    if (isObstacleInPath(*closestNode, newNode)){
        return;     // if there is osbtacle in path, return, do not add it to tree
    }

    Node* parent = new Node(*closestNode);
    newNode.setParent(parent);

    //std::cout << std::to_string(newNode.getParent()->getPosition()[0]) << std::endl;

    _nodes.push_back(newNode);
    
    // is it within range of end goal? 
        // yes -> done, back trace parents to start node 
        // no -> continue
}

void RRT::draw(){

    float LINE_WIDTH = 3.0;
    float NODE_RADIUS = 4.0;


    if (_nodes.size() < 2){
        return;
    }

    // DRAW NODES AND CONNECTIONS
    // =================================================================================
    for (int i = 1; i < _nodes.size(); i++){

        Node currentNode = _nodes.at(i);

        Node* parent = currentNode.getParent();
        std::vector<float> parent_position;

        if (parent != nullptr && (parent->getPosition().size() > 0)){
            parent_position = parent->getPosition();
        } else {
            continue;
        }

        std::vector<float> newNode_position = currentNode.getPosition();

        std::cout << "Node position: " << std::to_string(newNode_position.at(0)) << " " << std::to_string(newNode_position.at(1)) << std::endl;
        std::cout << "Parent position: " << std::to_string(parent_position.at(0)) << " " << std::to_string(parent_position.at(1)) << std::endl;

        float delta_x = parent_position.at(0) - newNode_position.at(0);
        float delta_y = parent_position.at(1) - newNode_position.at(1);

        float theta = (180 / 3.141592654) * atan2(delta_y, delta_x);

        float distance = getEuclideanDistance(parent_position, newNode_position);

        sf::RectangleShape line;
        line.setSize(sf::Vector2f(distance, LINE_WIDTH));
        line.setFillColor(sf::Color::Yellow);
        line.setPosition(sf::Vector2f(newNode_position.at(0), newNode_position.at(1)));
        line.setRotation(theta);

        _stateSpace.draw(line);

        sf::CircleShape point;
        point.setRadius(NODE_RADIUS);
        point.setFillColor(sf::Color::Red);
        point.setPosition(sf::Vector2f(newNode_position.at(0), newNode_position.at(1)));

        _stateSpace.draw(point);
    }
    // =================================================================================


    // DRAW OBSTACLES
    // =================================================================================
    for (int i = 0; i < _obstacles.size(); i++){
       _stateSpace.draw(_obstacles.at(i));
    }
    // =================================================================================

    // DRAW START AND GOAL
    // =================================================================================
    sf::CircleShape start;
    sf::CircleShape goal;

    start.setFillColor(sf::Color::Cyan);
    int start_radius = 20;
    start.setRadius(20);
    start.setPosition(sf::Vector2f(_startPosition.at(0) - start_radius/2, _startPosition.at(1) - start_radius/2));

    goal.setFillColor(sf::Color::Green);
    goal.setRadius(_tolerance);
    goal.setPosition(sf::Vector2f(_endPosition.at(0), _endPosition.at(1)));

    _stateSpace.draw(start);
    _stateSpace.draw(goal);
    // =================================================================================


}