#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <memory>

#include "CarState.hpp"
#include "Coordinates2D.hpp"
#include "TargetSpeed.hpp"


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
   using SpeedPoints = Coordinates2D<NWayPoints>;

   Trajectory();
   ~Trajectory();
   Trajectory(const Trajectory&) = delete;
   Trajectory(const Trajectory&&) = delete;

   // calculates spline and path points
   const PathPoints & calculateLanePath(const CarState & carState, const CarState & targetCarState, const TargetSpeed & targetSpeed, const Track & track, const double updateInterval);
   const PathPoints & calculateLinearPath(const CarState & carState, const CarState & targetCarState, const Track & track, const double updateInterval);

   // set previous path points before calculation
   void insertPreviousPath(const std::vector<double> & xCoordinates, const std::vector<double> & yCoordinates)
   {
      m_pathPoints.insertPath(xCoordinates, yCoordinates);
   }

   const PathAxis & getPathAxisX() const { return m_pathPoints.getX(); }
   const PathAxis & getPathAxisY() const { return m_pathPoints.getY(); }

private:
   double getSpeed(const Coordinate2D & xyPos0,  const Coordinate2D & xyPos1, const Track &track, const double updateInterval);

   // points that describe a path
   PathPoints m_pathPoints;

   // way points where a path must lead through
   WayPoints m_wayPoints;

   // Speed Ancor Points
   SpeedPoints m_speedPoints;

   // Spline for this trajectory
   std::unique_ptr<NONAME::tk::spline> m_waySplinePtr;
   std::unique_ptr<NONAME::tk::spline> m_speedSplinePtr;
   //NONAME::tk::spline * m_spline;

};

#endif // TRAJECTORY_HPP
