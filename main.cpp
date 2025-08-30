#include <SFML/Graphics.hpp>
#include <SFML/System/Vector2.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <vector>
#include <cmath>


#include "rrt.h"


using namespace sf;
using namespace std;

// width in height, in grid cells
const int WIDTH = 1900;
const int HEIGHT = 1000;

vector<int> STATE_SPACE = {WIDTH, HEIGHT};      // render window state space, contains obstacles, start, and end position

const int GROWTH_FACTOR = 20;                  // growth factor (euclidean distance) of new nodes being added to tree
const float TOLERANCE = 40;                     // euclidean distance tolerance to end position
const int OBSTACLE_DECETION_SEGMENTS = 15;      // number of segments between new node and closest node to check for collision

const float BIAS_TOWARDS_GOAL = 0.6;

int main(){

  Vector2i mousePosition;

  bool rrt_running = false;
  int iteration = 0;

  // RENDER WINDOW
  // =========================================================
  RenderWindow renderWindow(VideoMode(WIDTH, HEIGHT), "RRT");
  renderWindow.setFramerateLimit(60);
  // =========================================================

  vector<float> start_pos = {-100.0, -100.0};
  vector<float> end_pos = {-100.0, -100.0};

  RRT rrt(renderWindow, GROWTH_FACTOR, start_pos, end_pos, TOLERANCE, OBSTACLE_DECETION_SEGMENTS, BIAS_TOWARDS_GOAL);

  Font font;
  FileInputStream fontIn;
  fontIn.open("slkscr.ttf");
  font.loadFromStream(fontIn);

  Text iterationText;
  iterationText.setFillColor(Color::White);
  iterationText.setPosition(10, 10);
  iterationText.setFont(font);
  iterationText.setCharacterSize(30);

  bool placing_object = false;
  bool start_placed = false;
  bool goal_placed = false;
  bool mouse_held = false;

  vector<RectangleShape> obstacle_previews;
  RectangleShape obstacle_preview;
  obstacle_preview.setFillColor(Color::Magenta);
  Vector2i obstacle_corner;

  bool sim_started = false;
  bool goal_reached = false;

  bool spacebar_held = false;

  CircleShape start;
  CircleShape end;
  Event renderWindowEvent;

  start.setFillColor(Color::Green);
  end.setFillColor(Color::Red);
  start.setRadius(10);
  end.setRadius(TOLERANCE);

  start.setPosition(Vector2f(-100.0, -100.0));
  end.setPosition(Vector2f(-100.0, -100.0));
    
  while(renderWindow.isOpen()){

    renderWindow.clear();

    mousePosition = Mouse::getPosition(renderWindow);

    if (Mouse::isButtonPressed(Mouse::Left)){
      if (!sim_started) {
        if (!start_placed) {
          start_pos = {float(mousePosition.x), float(mousePosition.y)};
          start.setPosition(Vector2f(start_pos[0] - start.getRadius(), start_pos[1] - start.getRadius()));
        } else if (!goal_placed) {
          end_pos = {float(mousePosition.x), float(mousePosition.y)};
          end.setPosition(Vector2f(end_pos[0] - TOLERANCE, end_pos[1] - TOLERANCE));
        } else if (!placing_object && !rrt_running && !sim_started){     // if not drawing an obstacle, sim hasn't started, and mouse pressed, begin drawing it
          obstacle_corner = mousePosition;
          obstacle_preview.setPosition(Vector2f(obstacle_corner.x, obstacle_corner.y));
        } else if (placing_object) {    // drawing obstacle
          obstacle_preview.setSize(Vector2f(mousePosition.x - obstacle_corner.x, mousePosition.y - obstacle_corner.y));
          renderWindow.draw(obstacle_preview);
        }
        placing_object = true;
        mouse_held = true;
      }
    } else if (placing_object) {      // if drawing an object and mouse released, freeze object in place (stop updating its position)
      if (!start_placed) {
        rrt.setStart(start_pos);
        start_placed = true;
      } else if (!goal_placed) {
        rrt.setGoal(end_pos);
        goal_placed = true;
      } else {    // drawing obscacle
        obstacle_previews.push_back(obstacle_preview);
        rrt.addObstacle(obstacle_preview);
        placing_object = false;
      }
      placing_object = false;
    } else {
      mouse_held = false;
    }

    // RRT ====================================================
    if (rrt_running && !goal_reached){
      iteration ++;
      rrt.update();

      goal_reached = rrt.isGoalReached();

      iterationText.setString("Iteration: " + to_string(iteration));
    }
    // =========================================================

    // KEYBOARD EVENTS =========================================
    if (Keyboard::isKeyPressed(Keyboard::Space) && !spacebar_held){   // space to pause / unpause
      sim_started = true;
      rrt_running = !rrt_running;
      spacebar_held = true;
    } else if (!Keyboard::isKeyPressed(Keyboard::Space)) {
      spacebar_held = false;
    }
    
    // =========================================================


    // CLOSE WINDOWS IF X PRESSED
    // =========================================================

    while(renderWindow.pollEvent(renderWindowEvent)){
      if(renderWindowEvent.type == Event::Closed){
        renderWindow.close();
      }
    }
    // =========================================================

    // DRAW OBSTACLE AND START/STOP PREVIEWS ===================
    if (!rrt_running){
      for (int i = 0; i < obstacle_previews.size(); i++){
        renderWindow.draw(obstacle_previews.at(i));
      }
      renderWindow.draw(start);
      renderWindow.draw(end);
    }
    // =========================================================

    rrt.draw();
    renderWindow.draw(iterationText);

    renderWindow.display();
  }

  return 0;
}
