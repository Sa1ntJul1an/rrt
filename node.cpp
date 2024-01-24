#include "node.h"

#include <vector>
#include <set>

Node::Node(){
    _position = {};
    _parent = nullptr;

    std::set<Node*> _children;
}

Node::Node (std::vector<float> position, Node* parent, std::set<Node*> children){
    _position = position;
    _parent = parent;
    _children = children;
};

Node* Node::getParent(){
    return _parent;
}

std::vector<float> Node::getPosition(){
    return _position;
}

void Node::setPosition(std::vector<float> position){
    _position = position;
}

void Node::setParent(Node* parent){
    _parent = parent;
}