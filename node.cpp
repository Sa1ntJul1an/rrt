#include "node.h"

#include <vector>
#include <set>

#include <iostream>

Node::Node(){
    _position = {0};
    _parent = nullptr;

    std::set<Node*> _children;
}

Node::Node (std::vector<float> position, Node* parent, std::set<Node*> children){
    _position = position;
    _parent = parent;
    _children = children;
};

Node::Node(const Node& node){
    this->_position = node.getPosition();
    this->_parent = node.getParent();
}

Node* Node::getParent() const{
    return this->_parent;
}

std::vector<float> Node::getPosition() const {
    return this->_position;
}

void Node::setPosition(std::vector<float> position){
    _position = position;
}

void Node::setParent(Node* parent){
    _parent = parent;
}