#ifndef RRT_HEADER
#define RRT_HEADER

#include <SFML/Graphics.hpp>
#include <vector>
#include <random>

class RRT{
    public:
        RRT(sf::RenderWindow& stateSpace, float, float, float);

        void addObstacle(sf::RectangleShape);

        void update();

    private:
        sf::RenderWindow& _stateSpace;
        std::vector<sf::RectangleShape> _obstacles;

        float _growthFactor;
        float _start;
        float _end;

        std::mt19937 _mt;


};




#endif /* !RRT_HEADER */