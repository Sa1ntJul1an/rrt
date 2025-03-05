#include "rrt.h"

#include <SFML/Graphics.hpp>
#include <vector>
#include <set>
#include <cmath>
#include <random>
#include <ctime>


RRT::RRT (sf::RenderWindow& stateSpace, float growthFactor, std::vector<float> start, std::vector<float> end, float tolerance, int obstacle_detection_segments, float bias_towards_goal)
  : _stateSpace(stateSpace) {
  
  std::vector<sf::RectangleShape> _obstacles = {};
  _growthFactor = growthFactor;
  _tolerance = tolerance;
  _obstacle_detection_segments = obstacle_detection_segments;

  _startPosition = start;
  _endPosition = end;

  _bias = bias_towards_goal;

  _mt = std::mt19937(time(nullptr));

  _lineColor = sf::Color::Yellow;

  _start.setFillColor(sf::Color::Green);
  int start_radius = 10;
  _start.setRadius(start_radius);
  _start.setPosition(sf::Vector2f(_startPosition.at(0) - start_radius/2.0, _startPosition.at(1) - start_radius/2.0));

  _goal.setFillColor(sf::Color(255, 0, 0, 150));
  _goal.setRadius(_tolerance);
  _goal.setPosition(sf::Vector2f(_endPosition.at(0) - _tolerance/2.0, _endPosition.at(1) - _tolerance/2.0));

  _point.setRadius(_NODE_RADIUS);
  _point.setFillColor(sf::Color::Red);
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

void RRT::setStart(std::vector<float> start_pos) {
  _startPosition = start_pos;
}

void RRT::setGoal(std::vector<float> end_pos) {
  _endPosition = end_pos;
}

void RRT::initializeSearch() {
  std::set<Node*> children;
  _startNode = Node(_startPosition, nullptr, children);

  _nodes.push_back(_startNode);
  
  _start.setPosition(sf::Vector2f(_startPosition.at(0) - _start.getRadius(), _startPosition.at(1) - _start.getRadius()));
  _goal.setPosition(sf::Vector2f(_endPosition.at(0) - _tolerance, _endPosition.at(1) - _tolerance));
}

void RRT::update(){
  if (!_searchStarted) {
    initializeSearch();
    _searchStarted = true;
  }

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

  _nodes.push_back(newNode);

  if (getEuclideanDistance(newNode.getPosition(), _endPosition) < _tolerance){
    _lineColor = sf::Color(70, 70, 0);
    _point.setFillColor(sf::Color(90, 0, 0));

    _goalReached = true;
    _endNode = newNode;
  }
}

bool RRT::isGoalReached(){
  return this->_goalReached;
}

void RRT::draw(){

  // DRAW START AND GOAL
  // =================================================================================
  _stateSpace.draw(_start);
  _stateSpace.draw(_goal);
  // =================================================================================

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

    sf::Vertex line[] =
    {
      sf::Vertex(sf::Vector2f(parent_position.at(0), parent_position.at(1)), _lineColor),
      sf::Vertex(sf::Vector2f(newNode_position.at(0), newNode_position.at(1)), _lineColor)
    };

    _stateSpace.draw(line, 2, sf::Lines);

    _point.setPosition(sf::Vector2f(newNode_position.at(0) - _NODE_RADIUS, newNode_position.at(1) - _NODE_RADIUS));

    if (_goalReached) {
      traceBackToStart(_endNode);
    }

    _stateSpace.draw(_point);
  }
  // =================================================================================


  // DRAW OBSTACLES
  // =================================================================================
  for (int i = 0; i < _obstacles.size(); i++){
     _stateSpace.draw(_obstacles.at(i));
  }
  // =================================================================================
}

void RRT::traceBackToStart(Node currentNode) {
  Node* parent = currentNode.getParent();

  while (parent != nullptr) {
    std::vector<float> parent_position = parent->getPosition();
    std::vector<float> currentNode_position = currentNode.getPosition();

    sf::Vertex line[] =
      {
        sf::Vertex(sf::Vector2f(parent_position.at(0), parent_position.at(1)), sf::Color::Cyan),
        sf::Vertex(sf::Vector2f(currentNode_position.at(0), currentNode_position.at(1)), sf::Color::Cyan)
      };

    _stateSpace.draw(line, 2, sf::Lines);

    currentNode = *parent;
    parent = currentNode.getParent();
  }
}
