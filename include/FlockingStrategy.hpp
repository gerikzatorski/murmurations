#ifndef __FLOCKINGSTRATEGY_HPP__
#define __FLOCKINGSTRATEGY_HPP__

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Boid.hpp"

namespace murmurations {
	
	enum class FlockEnum { Basic, Projection };

	class FlockingStrategy {
	public:
		virtual ~FlockingStrategy() = default;
		virtual void flock(std::vector<Boid>&) = 0;
	};

	class BasicFlocking : public FlockingStrategy {
	public:
		void flock(std::vector<Boid>&);
	};

	class ProjectionFlocking: public FlockingStrategy {
	/* https://www.pnas.org/content/111/29/10422 */
	public:
		void flock(std::vector<Boid>&);
	};

}

#endif