#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "Coordinates2D.hpp"

class Trajectory
{
public:
   static constexpr size_t NPathPoints = 50;
   using PathPoints = Coordinates2D<NPathPoints>;
   using PathAxis = Coordinates2D<NPathPoints>::Axis;

   void straight(const Coordinate2D & position);

   const PathAxis & getPathAxisX() {return m_pathPoints.getX(); }
   const PathAxis & getPathAxisY() {return m_pathPoints.getY(); }

private:
   Coordinates2D<50> m_pathPoints;
};

#endif // TRAJECTORY_HPP
