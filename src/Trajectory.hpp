#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "CarState.hpp"
#include "Coordinates2D.hpp"

class Trajectory
{
public:
   static constexpr size_t NPathPoints = 50;
   static constexpr size_t NWayPoints = 5;
   using PathPoints = Coordinates2D<NPathPoints>;
   using PathAxis = PathPoints::Axis;
   using WayPoints = Coordinates2D<NWayPoints>;

   void calculate(const CarState & carState, const CarState & targetCarState, const Track & track);

   void insertPreviousPath(const std::vector<double> & xCoordinates, const std::vector<double> & yCoordinates)
   {
      m_pathPoints.insert(xCoordinates, yCoordinates);
   }

   const PathAxis & getPathAxisX() {return m_pathPoints.getX(); }
   const PathAxis & getPathAxisY() {return m_pathPoints.getY(); }

private:
   // points that describe a path
   PathPoints m_pathPoints;

   // way points where a path must lead through
   WayPoints m_wayPoints;
};

#endif // TRAJECTORY_HPP
