#include "Flock.hpp"

murmurations::Flock::Flock(int n)
{

	// Add boids
	for (int i = 0; i < n; i++) {
		Eigen::Vector2d position = 2 * Eigen::Vector2d::Random(2);
		position = 100*(position + Eigen::Vector2d::Constant(4)); // 300-500
		Eigen::Vector2d velocity = Eigen::Vector2d::Random(2);

		boids.push_back(Boid(i, position, velocity));
	}

	//using namespace Eigen;
	//boids.push_back(Boid(0, Vector2d(100, 100), Vector2d(0, 0)));
	//boids.push_back(Boid(1, Vector2d(150, 100), Vector2d(0, 0)));
	//boids.push_back(Boid(2, Vector2d(100, 150), Vector2d(0, 0)));
}

void murmurations::Flock::print()
{
	for (auto& b : boids)
		b.print();
}

void murmurations::Flock::flock()
{
	Sleep(0);

	// Threaded approach
	//std::vector<std::thread> threads;
	//for (auto& b : boids)
	//	threads.push_back(std::thread ([&] { b.flock(boids); }));
	//for (auto& th : threads)
	//	th.join();

	// Non-threaded approach
	for (auto& b : boids)
		b.flock(boids);

	for (auto& b : boids)
		b.update();
}