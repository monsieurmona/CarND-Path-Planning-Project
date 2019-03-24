#ifndef PATHPLAN_HPP
#define PATHPLAN_HPP

#include "CarState.hpp"
#include "Environment.hpp"
#include "Trajectory.hpp"

class PathPlan
{
public:
   struct LaneSpeed
   {
      LaneSpeed(const size_t laneIdx, const double speedInMps, const double distanceToNextCar):
         m_laneIdx{laneIdx}, m_speedInMps{speedInMps}, m_distanceToNextCar{distanceToNextCar} {}

      size_t m_laneIdx;
      double m_speedInMps;
      double m_distanceToNextCar;

      // greater than, sorts from highest value to lowest
      bool operator()(const LaneSpeed & a, const LaneSpeed & b)
      {
         const double distanceWeight = 0.001;
         const double penaltyA = a.m_distanceToNextCar * distanceWeight;
         const double penaltyB = b.m_distanceToNextCar * distanceWeight;
         const double speedA = a.m_speedInMps + a.m_speedInMps * penaltyA;
         const double speedB = b.m_speedInMps + b.m_speedInMps * penaltyB;
         return speedA > speedB;
      }
   };

   PathPlan(const std::vector<double> & prevXCoordinates,
            const std::vector<double> & prevYCoordinates,
            const Environment & environment,
            const CarState & egoCarState,
            const double updateInterval);

   const Trajectory & getEgoTrajectory() const { return  m_egoTrajectory; }

private:
   void getLaneChangeOptions(const Environment & environment, std::vector<size_t> & possibleLanes) const;
   std::vector<LaneSpeed> getSortedLaneSpeed(const Environment & environment) const;
   bool isFasterLane(const LaneSpeed & desiredLane, const LaneSpeed & currentLane, const std::vector<size_t> & possibleOptions) const;
   bool isWayFree(const CarState & egoCarState, const LaneSpeed & currentLane, const LaneSpeed & desiredLane, const Environment & environment) const;

   Trajectory m_egoTrajectory;
};

#endif // PATHPLAN_HPP
