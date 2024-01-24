#ifndef NODE_HEADER
#define NODE_HEADER

#include <vector>
#include <set>

class Node{
    public:
        Node();
        Node(std::vector<float>, Node*, std::set<Node*>);

        Node* getParent();
        std::vector<float> getPosition();
        void setPosition(std::vector<float>);
        void setParent(Node*);

    private:
        std::vector<float> _position;
        Node* _parent;
        std::set<Node*> _children;
};

#endif /* !NODE_HEADER */