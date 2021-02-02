#include "Flock.hpp"

murmurations::Flock::Flock(int n) {

	// Add boids
	for (int i = 0; i < n; i++) {
		Eigen::Vector2d position = Eigen::Vector2d::Random(2);
		position = 100*(position + Eigen::Vector2d::Constant(2)); // now 100-300
		Eigen::Vector2d velocity = Eigen::Vector2d::Random(2);

		boids.push_back(Boid(i, position, velocity));
	}

	//using namespace Eigen;
	//boids.push_back(Boid(0, Vector2d(100, 100), Vector2d(0, 0)));
	//boids.push_back(Boid(1, Vector2d(150, 100), Vector2d(0, 0)));
	//boids.push_back(Boid(2, Vector2d(100, 150), Vector2d(0, 0)));

	_strategy = std::make_unique<murmurations::ProjectionFlocking>();
}

void murmurations::Flock::print() {
	for (auto& b : boids) {
		b.print();
	}
}

void murmurations::Flock::flock() {
	Sleep(0);

	_strategy->flock(boids);
	for (auto& b : boids)
		b.update();
}

void murmurations::Flock::setStrategy(murmurations::FlockEnum strategyenum) {
	switch (strategyenum) {
	case murmurations::FlockEnum::Basic: _strategy = std::make_unique<murmurations::BasicFlocking>(); break;
	case murmurations::FlockEnum::Projection: _strategy = std::make_unique<murmurations::ProjectionFlocking>(); break;
	}
}