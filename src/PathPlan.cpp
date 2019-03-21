#include <algorithm>

#include "PathPlan.hpp"

PathPlan::PathPlan(const std::vector<double> & prevXCoordinates,
                   const std::vector<double> & prevYCoordinates,
                   const Environment & environment,
                   const CarState & egoCarState,
                   const double updateInterval)
{
   m_egoTrajectory.insertPreviousPath(prevXCoordinates, prevYCoordinates);

   std::vector<size_t> possibleLanes;
   getLaneChangeOptions(environment, possibleLanes);

   const std::vector<LaneSpeed> lanesSpeed = getSortedLaneSpeed(environment);

   const size_t currentLaneIdx = environment.getCurrentLaneIdx();
   const LaneSpeed currentLane(currentLaneIdx, environment.getLanesSpeedInMps()[currentLaneIdx]);

   std::cout << "Lanes";
   for (const LaneSpeed & desiredLane : lanesSpeed)
   {
      std::cout << " Speed:" << desiredLane.m_speedInMps << " id:" << desiredLane.m_laneIdx;
   }

   for (const LaneSpeed & desiredLane : lanesSpeed)
   {
      if (!isFasterLane(desiredLane, currentLane, possibleLanes) && desiredLane.m_laneIdx != currentLane.m_laneIdx)
      {
         continue;
      }

      const double maxSpeed = CarState::convertMilesPerHourToMetersPerSecond(49.0);
      const double desiredSpeed = desiredLane.m_speedInMps > maxSpeed ? maxSpeed : desiredLane.m_speedInMps;

      CarState desiredCarState(
               static_cast<int>(desiredLane.m_laneIdx),
               90 + egoCarState.m_carPositionSD.getS(),
               desiredSpeed,
               environment.getTrack());

      std::cout << " Desired Speed:" << desiredSpeed << std::endl;

      m_egoTrajectory.calculateLanePath(egoCarState, desiredCarState, environment.getTrack(), updateInterval);
      break;
   }
}

void PathPlan::getLaneChangeOptions(const Environment & environment, std::vector<size_t> & possibleLanes) const
{
   const size_t currentLaneIdx = environment.getCurrentLaneIdx();
   possibleLanes.reserve(2);

   if (currentLaneIdx > 0)
   {
      possibleLanes.push_back(currentLaneIdx - 1);
   }

   possibleLanes.push_back(currentLaneIdx);

   if (currentLaneIdx + 1 < environment.getLaneCount())
   {
      possibleLanes.push_back(currentLaneIdx + 1);
   }
}


std::vector<PathPlan::LaneSpeed> PathPlan::getSortedLaneSpeed(const Environment & environment) const
{
   const std::vector<double> & unsortedLanesSpeedInMps = environment.getLanesSpeedInMps();
   std::vector<LaneSpeed> lanesSpeedInMps;
   lanesSpeedInMps.reserve(unsortedLanesSpeedInMps.size());

   for  (size_t i = 0; i < unsortedLanesSpeedInMps.size(); ++i)
   {
      lanesSpeedInMps.emplace_back(LaneSpeed(i, unsortedLanesSpeedInMps[i]));
   }

   if (lanesSpeedInMps.size() > 0)
   {
      std::sort(lanesSpeedInMps.begin(), lanesSpeedInMps.end(), *lanesSpeedInMps.begin());
   }

   return lanesSpeedInMps;
}

bool PathPlan::isFasterLane(const LaneSpeed & desiredLane, const LaneSpeed & currentLane, const std::vector<size_t> & possibleOptions) const
{
   const bool isFaster = desiredLane.m_speedInMps > currentLane.m_speedInMps;

   bool isPossible = false;

   if (isFaster)
   {
      for (const auto & possibleOption : possibleOptions)
      {
         if (desiredLane.m_laneIdx == possibleOption)
         {
            isPossible = true;
            break;
         }
      }
   }

   return isPossible && isFaster;
}
