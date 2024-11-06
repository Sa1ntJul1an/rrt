#ifndef RRT_HEADER
#define RRT_HEADER

#include <vector>
#include <random>
#include <SFML/Graphics.hpp>

#include "node.h"

class RRT{
    public:
        RRT(sf::RenderWindow&, float, std::vector<float>, std::vector<float>, float, int, float);

        void addObstacle(sf::RectangleShape);
        float getEuclideanDistance(std::vector<float>, std::vector<float>);
        
        void draw();
        void update();

        bool isGoalReached();

    private:
        sf::RenderWindow& _stateSpace;
        std::vector<sf::RectangleShape> _obstacles;

        const float _NODE_RADIUS = 4.0;
        sf::Color _lineColor;
        sf::Color _nodeColor;

        float _growthFactor;
        float _tolerance;
        int _obstacle_detection_segments;
        std::vector<float> _startPosition;
        std::vector<float> _endPosition;

        Node _startNode;
        Node _endNode;
        std::vector<Node> _nodes;

        std::mt19937 _mt;

        bool _goalReached = false;
        float _bias;

        // SFML OBJECTS
        sf::CircleShape _point;
        sf::CircleShape _start;
        sf::CircleShape _goal;

        // private functions
        Node sampleStateSpace();
        std::vector<float> getUnitVector(std::vector<float>, std::vector<float>);
        void normalizeNodeToGrowthFactor(Node, Node&);
        Node* getClosestNode(Node);
        bool isCollision(std::vector<float>);
        bool isObstacleInPath(Node, Node);
        void traceBackToStart(Node);
};

#endif /* !RRT_HEADER */