#ifndef RRT_HEADER
#define RRT_HEADER

#include <vector>
#include <random>
#include <SFML/Graphics.hpp>

#include "node.h"

class RRT{
    public:
        RRT(sf::RenderWindow&, float, std::vector<float>, std::vector<float>, float, int);

        void addObstacle(sf::RectangleShape);
        float getEuclideanDistance(std::vector<float>, std::vector<float>);

        void update();

    private:
        sf::RenderWindow& _stateSpace;
        std::vector<sf::RectangleShape> _obstacles;

        float _growthFactor;
        float _tolerance;
        int _obstacle_detection_segments;
        std::vector<float> _startPosition;
        std::vector<float> _endPosition;

        Node _startNode;
        std::vector<Node> _nodes;

        std::mt19937 _mt;

        Node sampleStateSpace();
        void normalizeNodeToGrowthFactor(Node, Node&);
        Node* getClosestNode(Node);
        bool isCollision(std::vector<float>);
        bool isObstacleInPath(Node, Node);
        void draw();

};

#endif /* !RRT_HEADER */