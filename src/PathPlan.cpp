/** Author: Mario Lüder
 * Date: 2019-03-24
 */

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
   const LaneSpeed currentLane(
            currentLaneIdx,
            environment.getLanesSpeedInMps()[currentLaneIdx],
            environment.getDistanceToNextCar(currentLaneIdx));

   std::cout << "Current Lane Index:" << currentLane.m_laneIdx << std::endl;
   std::cout << "Lanes";

   for (const LaneSpeed & desiredLane : lanesSpeed)
   {
      std::cout << " Speed:" << desiredLane.m_speedInMps << " id:" << desiredLane.m_laneIdx;
   }
   std::cout << std::endl;

   for (const LaneSpeed & desiredLane : lanesSpeed)
   {
      if (desiredLane.m_laneIdx != currentLane.m_laneIdx)
      {
         // lane is not an option, if the lane is not faster than curren lane
         if (!isFasterLane(desiredLane, currentLane, possibleLanes))
         {
            continue;
         }

         if (!isWayFree(egoCarState, currentLane, desiredLane, environment))
         {
            continue;
         }
      }

      const int nLanesToChange = abs(static_cast<int>(desiredLane.m_laneIdx) - static_cast<int>(currentLane.m_laneIdx));
      const double maxSpeed = CarState::convertMilesPerHourToMetersPerSecond(49.0);

      const double laneSpeedInMps = desiredLane.m_speedInMps > maxSpeed ? maxSpeed : desiredLane.m_speedInMps;
      Environment::DistanceSpeed distanceSpeedFront = environment.getClosestCarsInLane(desiredLane.m_laneIdx).m_front;
      const double desiredSpeedInMps = environment.predictLaneSpeed(egoCarState, desiredLane.m_laneIdx, 1.0);

      std::cout << " Lane Speed:" << laneSpeedInMps
                << " nLane changes:" << nLanesToChange
                << " Desired Speed:" << desiredSpeedInMps
                << std::endl << std::endl;

      const double predictionHorizonInSecond = 1.0;
      double predictedDistance =
            predictionHorizonInSecond * distanceSpeedFront.m_speedInMps
            + distanceSpeedFront.m_distanceInM
            - CarState::convertMetersPerSecondToMilesPerHour(desiredSpeedInMps) * environment.m_safefyDistanceFactor;

      double laneChangeHorizon = desiredSpeedInMps * 4.8;

      if (laneChangeHorizon < 20)
      {
         laneChangeHorizon = 20;
      }

      if (nLanesToChange > 1)
      {
         laneChangeHorizon = (nLanesToChange + 1) * laneChangeHorizon;
      }

      if (predictedDistance > laneChangeHorizon)
      {
         predictedDistance = laneChangeHorizon;
      }

      if (predictedDistance < 1)
      {
         predictedDistance = 1;
      }

      CarState desiredCarState(
               static_cast<int>(desiredLane.m_laneIdx),
               laneChangeHorizon + egoCarState.m_carPositionSD.getS(),
               desiredSpeedInMps,
               environment.getTrack());

      TargetSpeed targetSpeed(desiredSpeedInMps, predictedDistance);

      m_egoTrajectory.calculateLanePath(egoCarState, desiredCarState, targetSpeed, environment.getTrack(), updateInterval);
      break;
   }
}

void PathPlan::getLaneChangeOptions(const Environment & environment, std::vector<size_t> & possibleLanes) const
{
   const size_t laneCount = environment.getLanesSpeedInMps().size();
   possibleLanes.reserve(laneCount);
   possibleLanes.clear();

   for (size_t i = 0; i < laneCount; ++i)
   {
      possibleLanes.push_back(i);
   }
}


std::vector<PathPlan::LaneSpeed> PathPlan::getSortedLaneSpeed(const Environment & environment) const
{
   const std::vector<double> & unsortedLanesSpeedInMps = environment.getLanesSpeedInMps();
   std::vector<LaneSpeed> lanesSpeedInMps;
   lanesSpeedInMps.reserve(unsortedLanesSpeedInMps.size());

   for  (size_t i = 0; i < unsortedLanesSpeedInMps.size(); ++i)
   {
      lanesSpeedInMps.emplace_back(LaneSpeed(i, unsortedLanesSpeedInMps[i], environment.getDistanceToNextCar(i)));
   }

   if (lanesSpeedInMps.size() > 0)
   {
      std::sort(lanesSpeedInMps.begin(), lanesSpeedInMps.end(), *lanesSpeedInMps.begin());
   }

   return lanesSpeedInMps;
}

bool PathPlan::isFasterLane(const LaneSpeed & desiredLane, const LaneSpeed & currentLane, const std::vector<size_t> & possibleOptions) const
{
   const bool isFaster = desiredLane.m_speedInMps > (currentLane.m_speedInMps + 0.4);

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

bool PathPlan::isWayFree(const CarState & egoCarState, const LaneSpeed & currentLane, const LaneSpeed & desiredLane, const Environment & environment) const
{
   if (currentLane.m_laneIdx == desiredLane.m_laneIdx)
   {
      return true;
   }

   const int desiredLaneIdx = static_cast<int>(desiredLane.m_laneIdx);
   const int currentLaneIdx = static_cast<int>(currentLane.m_laneIdx);
   const int laneDistance = desiredLaneIdx - currentLaneIdx;
   const double carSpeedInMps = egoCarState.getSpeedInMetersPerSecond();

   const int step = laneDistance / abs(laneDistance);
   int nextLane = currentLaneIdx;

   while (nextLane != desiredLaneIdx)
   {
      nextLane += step;

      const Environment::ClosestCarsInLane closestCarsInLane = environment.getClosestCarsInLane(nextLane);
      std::cout << "Ego Car Speed:" << carSpeedInMps << " Desired Lane Speed:" << desiredLane.m_speedInMps << std::endl;

      // check if there is enough distance to the back
      // use the slowest speed
      const double slowestSpeedInMps = carSpeedInMps < desiredLane.m_speedInMps ?
               carSpeedInMps : desiredLane.m_speedInMps;

      const double speedBackCarInMps = closestCarsInLane.m_back.m_speedInMps;
      const double speedDiffToBackInMps = speedBackCarInMps - slowestSpeedInMps;
      const double speedBackCarInMph = CarState::convertMetersPerSecondToMilesPerHour(speedBackCarInMps);
      const double speedDiffToBackInMph = CarState::convertMetersPerSecondToMilesPerHour(speedDiffToBackInMps);

      double safetyDistanceBack = (speedBackCarInMph + speedDiffToBackInMph) * environment.m_safefyDistanceFactor;

      std::cout << "Safety Distance Back:" << safetyDistanceBack
                << " Distance Back:" << closestCarsInLane.m_back.m_distanceInM
                << " Lane:" << nextLane
                << std::endl;

      if (safetyDistanceBack < 3)
      {
         safetyDistanceBack = 3.0;
      }

      if (safetyDistanceBack > closestCarsInLane.m_back.m_distanceInM)
      {
         // the distance to the back is too small
         return false;
      }

      // check if there is enough space to the front
      const double fastestSpeedInMps = carSpeedInMps > desiredLane.m_speedInMps ?
               carSpeedInMps : desiredLane.m_speedInMps;
      const double fastestSpeedInMph = CarState::convertMetersPerSecondToMilesPerHour(fastestSpeedInMps);
      double safetyDistanceFront = (fastestSpeedInMph) * environment.m_safefyDistanceFactor;

      std::cout << "Safety Distance Front:" << safetyDistanceFront
                << " Distance Front:" << closestCarsInLane.m_front.m_distanceInM
                << " Lane:" << nextLane
                << std::endl;

      if (safetyDistanceFront < 3)
      {
         safetyDistanceFront = 3.0;
      }

      if (safetyDistanceFront > 100000)
      {
         return false;
      }

      if (safetyDistanceFront > closestCarsInLane.m_front.m_distanceInM)
      {
         // the distance to the back is too small
         return false;
      }
   }

   return true;
}
