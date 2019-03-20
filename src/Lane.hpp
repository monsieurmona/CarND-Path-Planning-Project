#ifndef LANE_HPP
#define LANE_HPP

#include <cmath>

class Lane
{
public:
   Lane(double laneWidth) : m_laneWidth(laneWidth), m_halfLaneWidth(laneWidth / 2.0) {}

   inline double getD(const int laneIdx) const
   {
      return m_halfLaneWidth + laneIdx * m_laneWidth;
   }

   inline int getLaneIdx(const double d)
   {
      return static_cast<int>(floor(d / m_laneWidth));
   }

private:
   double m_laneWidth;
   double m_halfLaneWidth;
};

#endif // LANE_HPP
