#include <iostream>
#include <SFML/Graphics.hpp>
#include <chrono>

#include "Boid.hpp"
#include "Flock.hpp"
#include "FlockingStrategy.hpp"

using namespace std;
using namespace murmurations;

int main() {
    srand((unsigned int)time(0));
	cout << "Starting murmur program" << endl;

	// Setup
	Flock flock(40);
	int maxSteps = 200;
    sf::RenderWindow window(sf::VideoMode(800, 600), "My window");

    // Loop
    int step = 0;
    while (window.isOpen()) {
        printf("------ STEP %i ------\n", step);

        flock.print();
        auto start = chrono::high_resolution_clock::now();
        flock.flock();
        auto stop = chrono::high_resolution_clock::now();
        auto duration = chrono::duration_cast<chrono::microseconds>(stop - start);
        cout << "Time taken by function: " << duration.count() << " microseconds" << endl;

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }
        
        if (step == maxSteps) window.close();

        window.clear(sf::Color::Black);

        // draw stuff ...
        // TODO: threads
        for (auto& b : flock.boids) {
            sf::CircleShape shape(4.f);
            shape.setFillColor(sf::Color(100, 250, 50));
            shape.setPosition((float)b.position().x(), (float)b.position().y());
            window.draw(shape);
        }

        window.display();
        step++;
    }

	return 0;
}