#ifndef RRT_HEADER
#define RRT_HEADER

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>

#include "node.h"

class RRT{
    public:
        RRT(sf::RenderWindow& stateSpace, float, std::vector<float>, std::vector<float>);

        void addObstacle(sf::RectangleShape);

        void update();

    private:
        sf::RenderWindow& _stateSpace;
        std::vector<sf::RectangleShape> _obstacles;

        float _growthFactor;
        std::vector<float> _startPosition;
        std::vector<float> _endPosition;

        Node _startNode;

        std::vector<Node> _nodes;

        std::mt19937 _mt;

};

#endif /* !RRT_HEADER */