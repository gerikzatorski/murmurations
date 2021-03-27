#include <SFML/Graphics.hpp>
#include <chrono>
#include <iostream>

#include "Boid.hpp"
#include "Flock.hpp"

#define NUM_BOIDS 50
#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 800

using namespace std;
using namespace murmurations;

int main() {
  srand((unsigned int)time(0));
  cout << "Starting murmur program" << endl;

  // Setup
  Flock flock{};
  for (int i = 0; i < NUM_BOIDS; i++) {
    Eigen::Vector2d position = Eigen::Vector2d::Random(2);
    position = 100 * (position + Eigen::Vector2d::Constant(4));
    Eigen::Vector2d velocity = Eigen::Vector2d::Random(2);

    Boid b = Boid(i, position, velocity);
    flock.boids.push_back(b);
  }
  int maxSteps = 700;
  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT),
                          "My window");

  // Loop
  int step = 0;
  while (window.isOpen()) {
    printf("------ STEP %i ------\n", step);

    // flock.print();
    auto start = chrono::high_resolution_clock::now();
    flock.flock();
    auto stop = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
    cout << "Time taken by function: " << duration.count() << " microseconds"
         << endl;

    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }

    if (step == maxSteps)
      window.close();

    window.clear(sf::Color::Black);

    // draw stuff ...
    for (auto &b : flock.boids) {
      sf::CircleShape shape(3.f);
      shape.setFillColor(sf::Color(100, 250, 50));
      // float x = (float)(b.position.x() - flock.center().x()) + WINDOW_WIDTH
      // / 2; float y = (float)(b.position.y() - flock.center().y()) +
      // WINDOW_HEIGHT / 2;
      float x = (float)b.position.x();
      float y = (float)b.position.y();
      shape.setPosition(x, y);
      window.draw(shape);
    }

    window.display();
    step++;
  }

  return 0;
}
