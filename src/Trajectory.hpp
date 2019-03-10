#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <memory>

#include "CarState.hpp"
#include "Coordinates2D.hpp"


namespace NONAME {
   namespace tk {
      class spline;
   }

}

class Trajectory
{
public:
   static constexpr size_t NPathPoints = 50;
   static constexpr size_t NWayPoints = 5;
   using PathPoints = Coordinates2D<NPathPoints>;
   using PathAxis = PathPoints::Axis;
   using WayPoints = Coordinates2D<NWayPoints>;

   Trajectory();
   ~Trajectory();

   // calculates spline and path points
   void calculate(const CarState & carState, const CarState & targetCarState, const Track & track, const double updateInterval);

   // set previous path points before calculation
   void insertPreviousPath(const std::vector<double> & xCoordinates, const std::vector<double> & yCoordinates)
   {
      m_pathPoints.insertPath(xCoordinates, yCoordinates);
   }

   const PathAxis & getPathAxisX() const { return m_pathPoints.getX(); }
   const PathAxis & getPathAxisY() const { return m_pathPoints.getY(); }

private:
   // points that describe a path
   PathPoints m_pathPoints;

   // way points where a path must lead through
   WayPoints m_wayPoints;

   // Spline for this trajectory
   std::unique_ptr<NONAME::tk::spline> m_splinePtr;
   //NONAME::tk::spline * m_spline;

};

#endif // TRAJECTORY_HPP
