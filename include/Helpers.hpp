#ifndef __HELPERS_HPP__
#define __HELPERS_HPP__

#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace murmurations {

	class Neighbor
	{
	public:
		Neighbor(int id, double distance)
			: _id(id)
			, _distance(distance)
		{}

		double distance() const;
		int id();
		friend bool operator < (const Neighbor& lhs, const Neighbor& rhs);
		friend bool operator > (const Neighbor& lhs, const Neighbor& rhs);

	private:
		int _id;
		double _distance;
	};
	class Event
	{
	public:
		Event(double theta, int id, bool frontEdge)
			: _theta(theta)
			, _frontEdge(frontEdge)
			, _id(id)
		{}

		int getID();
		double getTheta() const;
		bool isFront() const;

		friend bool operator < (const Event& lhs, const Event& rhs);
		friend bool operator > (const Event& lhs, const Event& rhs);
	
	private:
		int _id;
		double _theta;
		bool _frontEdge;
	};

	struct CustomCompare {
		bool operator()(const Event& lhs, const Event& rhs)
		{
			return lhs < rhs;
		}
	};

	Eigen::Vector2d AverageDirection(std::vector<Eigen::Vector2d>);
}

#endif