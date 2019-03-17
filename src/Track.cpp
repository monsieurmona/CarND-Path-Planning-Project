#include <limits>

#include "Track.hpp"

size_t Track::getClosestWaypoint(const Coordinate2D & xyCoordinate) const
{
   double closestLen = std::numeric_limits<double>::max(); //large number
   size_t closestWaypoint = 0;
   const size_t mapXLength = m_maps_x.size();

   for (size_t i = 0; i < mapXLength; ++i) {
      const Coordinate2D mapCoordinate(m_maps_x[i], m_maps_y[i]);
      const double distance = mapCoordinate.distance(xyCoordinate);
      if (distance < closestLen) {
         closestLen = distance;
         closestWaypoint = i;
      }
   }

   return closestWaypoint;
}

size_t Track::getNextWaypoint(const Coordinate2D & xyCoordinate, const double theta) const
{
   size_t closestWaypoint = getClosestWaypoint(xyCoordinate);
   const Coordinate2D mapCoordinate(m_maps_x[closestWaypoint], m_maps_y[closestWaypoint]);
   const double heading = mapCoordinate.headingFrom(xyCoordinate);

   double angle = fabs(theta - heading);
   angle = std::min(2 * M_PI - angle, angle);
   const size_t mapXSize = m_maps_x.size();

   if (angle > M_PI/2) {
      ++closestWaypoint;
      if (closestWaypoint == mapXSize) {
         closestWaypoint = 0;
      }
   }

   return closestWaypoint;
}

Coordinate2D Track::convertToSDtoXY(const Coordinate2D & sdCoordinate) const
{
   const double s = sdCoordinate.getS();
   const double d = sdCoordinate.getD();

   const int length_map_s = static_cast<int>(m_maps_s.size());
   const int length_map_x = static_cast<int>(m_maps_x.size());

   // from helper.h
   int prev_wp_signed = -1;

   while (s > m_maps_s[static_cast<size_t>(prev_wp_signed+1)] && (prev_wp_signed < (length_map_s-1))) {
      ++prev_wp_signed;
   }

   const size_t wp2 = static_cast<size_t>((prev_wp_signed+1)%length_map_x);
   const size_t prev_wp = static_cast<size_t>(prev_wp_signed);

   const Coordinate2D prevWpCoordinate(m_maps_x[prev_wp], m_maps_y[prev_wp]);
   const Coordinate2D wp2Coordinate(m_maps_x[wp2], m_maps_y[wp2]);
   const double heading = wp2Coordinate.headingFrom(prevWpCoordinate);

   // the x,y,s along the segment
   const double seg_s = (s - m_maps_s[prev_wp]);

   const Coordinate2D segVectorPart(seg_s * cos(heading), seg_s * sin(heading));
   const Coordinate2D seg = prevWpCoordinate + segVectorPart;

   const double perp_heading = heading - M_PI / 2;

   const Coordinate2D dVector(d * cos(perp_heading), d * sin(perp_heading));
   const Coordinate2D xyCoordinate = seg + dVector;

   return xyCoordinate;
}

Coordinate2D Track::convertXYtoSD(const Coordinate2D &xyCoordinate, const double theta) const
{
   int next_wp_signed = static_cast<int>(getNextWaypoint(xyCoordinate, theta));
   int prev_wp_signed = next_wp_signed - 1;

   if (next_wp_signed == 0) {
      prev_wp_signed = getXLength() - 1;
   }

   const size_t next_wp = static_cast<size_t>(next_wp_signed);
   const size_t prev_wp = static_cast<size_t>(prev_wp_signed);

   const Coordinate2D nextWpCoordinate(m_maps_x[next_wp], m_maps_y[next_wp]);
   const Coordinate2D prevWpCoordinate(m_maps_x[prev_wp], m_maps_y[prev_wp]);
   const Coordinate2D nCoordinate = nextWpCoordinate - prevWpCoordinate;
   const Coordinate2D xCoordinate = xyCoordinate - prevWpCoordinate;

   // find the projection of x onto n
   const Coordinate2D projection = xCoordinate.projection(nCoordinate);
   double frenet_d = xCoordinate.distance(projection);

   //see if d value is positive or negative by comparing it to a center point
   const Coordinate2D center(1000.0 - prevWpCoordinate.getX(), 2000.0 - prevWpCoordinate.getY());
   const double centerToPos = center.distance(xCoordinate);
   const double centerToRef = center.distance(projection);

   if (centerToPos <= centerToRef) {
      frenet_d *= -1;
   }

   // calculate s value
   double frenet_s = 0;
   for (size_t i = 0; i < prev_wp; ++i) {
      const Coordinate2D current(m_maps_x[i], m_maps_y[i]);
      const Coordinate2D next(m_maps_x[i + 1], m_maps_y[i + 1]);
      frenet_s += current.distance(next);
   }

   frenet_s += projection.distance();

   return {frenet_s,frenet_d};
}
