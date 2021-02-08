#include "Flock.hpp"

murmurations::Flock::Flock(int n)
{
	// Add boids
	for (int i = 0; i < n; i++) {
		Eigen::Vector2d position = Eigen::Vector2d::Random(2);
		position = 100*(position + Eigen::Vector2d::Constant(4));
		Eigen::Vector2d velocity = Eigen::Vector2d::Random(2);

		boids.push_back(Boid(i, basic, position, velocity));
	}

	//using namespace Eigen;
	//boids.push_back(Boid(0, basic, Eigen::Vector2d(200, 300), Eigen::Vector2d(0, 0)));
	//boids.push_back(Boid(1, basic, Eigen::Vector2d(300, 300), Eigen::Vector2d(0, 0)));
	//boids.push_back(Boid(2, basic, Eigen::Vector2d(400, 300), Eigen::Vector2d(0, 0)));
	//boids.push_back(Boid(3, basic, Eigen::Vector2d(300, 400), Eigen::Vector2d(0, 0)));
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

Eigen::Vector2d murmurations::Flock::center()
{
	Eigen::Vector2d res = Eigen::Vector2d::Constant(0);
	for (auto& b : boids)
		res += b.position();
	return res / boids.size();
}
