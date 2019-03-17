#ifndef LANE_HPP
#define LANE_HPP


class Lane
{
public:
   Lane(double laneWidth) : m_laneWidth(laneWidth), m_halfLaneWidth(laneWidth / 2.0) {}

   double getD(const int laneIdx) const
   {
      return m_halfLaneWidth + laneIdx * m_laneWidth;
   }
   double m_laneWidth;
   double m_halfLaneWidth;
};

#endif // LANE_HPP
