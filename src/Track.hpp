#ifndef TRACK_HPP
#define TRACK_HPP

#include <vector>

#include "Coordinate2D.hpp"

class Track
{
public:
   Track(const std::vector<double> &maps_s,
         const std::vector<double> &maps_x,
         const std::vector<double> &maps_y)
      : m_maps_s(maps_s)
      , m_maps_x(maps_x)
      , m_maps_y(maps_y)
   {}

   int getSLength() const { return static_cast<int>(m_maps_s.size());}
   int getXLength() const { return static_cast<int>(m_maps_x.size());}
   int getYLength() const { return static_cast<int>(m_maps_y.size());}


   // Calculate closest waypoint to current x, y position
   size_t getClosestWaypoint(const Coordinate2D & xyCoordinate) const;

   // Returns next waypoint of the closest waypoint
   size_t getNextWaypoint(const Coordinate2D & xyCoordinate, const double theta) const;

   Coordinate2D convertToSDtoXY(const Coordinate2D & sdCoordinate) const;
   Coordinate2D convertXYtoSD(const Coordinate2D & xyCoordinate, const double theta) const;

private:
   const std::vector<double> &m_maps_s;
   const std::vector<double> &m_maps_x;
   const std::vector<double> &m_maps_y;
};

#endif // TRACK_HPP
