#include <cmath>

#include "Environment.hpp"

void Environment::setEnvironment(const std::vector<std::vector<double>> & sensorFusion, const CarState & egoCarState)
{
   const size_t nCars = sensorFusion.size();

   // as we do simple prediction for now, erase previous states
   m_vehicles.clear();

   for (size_t carIdx = 0; carIdx < nCars; ++carIdx)
   {
      const std::vector<double> & carData = sensorFusion[carIdx];

      const int id = static_cast<int>(carData[0]);
      const double x = carData[1];
      const double y = carData[2];
      const double vx = carData[3];
      const double vy = carData[4];
      const double s = carData[5];
      const double d = carData[6];

      const double v = sqrt(vx * vx + vy * vy);
      const double yaw = atan2(vy, vx);

      addVehicle(id, CarState(x, y, s, d, yaw, v));
   }

   calculateLaneSpeed(egoCarState);

   const int laneIdx = m_lane.getLaneIdx(egoCarState.m_carPositionSD.getD());

   if (laneIdx >= 0 || laneIdx < static_cast<int>(m_nLanes))
   {
      m_currentLaneIdx = static_cast<size_t>(laneIdx);
   }

   m_closestCarInLanes.reserve(m_nLanes);

   for (size_t i = 0; i < m_nLanes; ++i)
   {
      ClosestCarsInLane closestCarInLane = getClosestCarsInLane(egoCarState, static_cast<int>(i));
      m_closestCarInLanes.push_back(closestCarInLane);
   }
}

void Environment::predict(const double_t horizon, const double updateInterval)
{
   for (Vehicle & vehicle : m_vehicles)
   {
      vehicle.predict(m_track, horizon, updateInterval);
   }
}

void Environment::calculateLaneSpeed(const CarState & egoCarState)
{
   assert(m_nLanes > 0);
   m_lanesSpeedInMps.resize(static_cast<size_t>(m_nLanes), m_maxSpeedInMps);
   m_nextCarDistance.resize(static_cast<size_t>(m_nLanes), std::numeric_limits<double>::max());

   for (const Vehicle & vehicle : m_vehicles)
   {
      const Coordinate2D & vehicleSdCoordinates = vehicle.getCarState().m_carPositionSD;
      const double distance = m_track.sDistance(egoCarState.m_carPositionSD.getS(), vehicleSdCoordinates.getS());

      if (distance < 0)
      {
         // if a car is behind, we don't want to count it
         continue;
      }

      const int laneIdx = m_lane.getLaneIdx(vehicleSdCoordinates.getD());

      if (laneIdx < 0 || laneIdx >= static_cast<int>(m_nLanes))
      {
         // car shall be on ego vehicle side of the road
         continue;
      }

      double & minNextCarDistance = m_nextCarDistance[static_cast<size_t>(laneIdx)];

      if (minNextCarDistance > distance)
      {
         minNextCarDistance = distance;

         double & laneSpeedInMps = m_lanesSpeedInMps[static_cast<size_t>(laneIdx)];
         // const double vehicleSpeedInMps = vehicle.getCarState().getSpeedInMetersPerSecond();

         const double safetyDistance = 5;
         const double maxPossibleVehicleSpeedInMps = CarState::convertMilesPerHourToMetersPerSecond(
                  (distance / m_safefyDistanceFactor) - safetyDistance);
         double vehicleSpeedInMps = maxPossibleVehicleSpeedInMps > m_maxSpeedInMps ? m_maxSpeedInMps : maxPossibleVehicleSpeedInMps;

         if (vehicleSpeedInMps < 0)
         {
            vehicleSpeedInMps = 0;
         }

         if (laneSpeedInMps > vehicleSpeedInMps)
         {
            laneSpeedInMps = vehicleSpeedInMps;
         }
      }
   }
}

Environment::ClosestCarsInLane Environment::getClosestCarsInLane(const CarState & egoCarPosition, const int laneIdx) const
{
   ClosestCarsInLane closestCarsInLane;

   for (const Vehicle & vehicle : m_vehicles)
   {
      const Coordinate2D & carPosSD = vehicle.getCarState().m_carPositionSD;
      const int vehicleLane = m_lane.getLaneIdx(carPosSD.getD());

      if (vehicleLane != laneIdx)
      {
         continue;
      }

      const double distanceToNextCar = m_track.sDistance(
               egoCarPosition.m_carPositionSD.getS(), carPosSD.getS());

      const double distanceToNextCarAbs = abs(distanceToNextCar);

      DistanceSpeed * distanceSpeed = nullptr;

      if (distanceToNextCar < 0)
      {
         distanceSpeed = &closestCarsInLane.m_back;
      }
      else
      {
         distanceSpeed = &closestCarsInLane.m_front;
      }

      if (distanceSpeed->m_distanceInM > distanceToNextCarAbs)
      {
         distanceSpeed->m_distanceInM = distanceToNextCarAbs;
         distanceSpeed->m_speedInMps = vehicle.getCarState().getSpeedInMetersPerSecond();
      }
   }

   std::cout << "Closest Cars in Lane:" << laneIdx
             << " | Front Distance:" << closestCarsInLane.m_front.m_distanceInM  << " Speed:" << closestCarsInLane.m_front.m_speedInMps
             << " | Back Distance:" << closestCarsInLane.m_back.m_distanceInM  << " Speed:" << closestCarsInLane.m_back.m_speedInMps << std::endl;

   return closestCarsInLane;
}

double Environment::predictLaneSpeed(const CarState & egoCarState, const size_t laneIdx, const double t) const
{
   const ClosestCarsInLane & clostestCarsInLane = m_closestCarInLanes[laneIdx];

   const double predictedCarPos = clostestCarsInLane.m_front.m_speedInMps * t + clostestCarsInLane.m_front.m_distanceInM;
   const double predictedEgoCarPos = egoCarState.getSpeedInMetersPerSecond() * t;
   const double predictedDistance = predictedCarPos - predictedEgoCarPos;

   double predictedSpeedInMps = CarState::convertMilesPerHourToMetersPerSecond(
            predictedDistance / m_safefyDistanceFactor);

   if (predictedSpeedInMps > m_maxSpeedInMps)
   {
      predictedSpeedInMps = m_maxSpeedInMps;
   }

   if (predictedSpeedInMps < 0)
   {
      predictedSpeedInMps = 0;
   }

   return  predictedSpeedInMps;
}
