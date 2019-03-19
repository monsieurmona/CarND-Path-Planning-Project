#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>

#include "Track.hpp"
#include "Vehicle.hpp"

class Environment
{
public:
   Environment(int nLanes, const Lane & lane ) : m_nLanes{nLanes}, m_lane{lane} {}

   void setEnvironment(const std::vector<std::vector<double>> & sensorFusion);
   void predict(const Track & track, const double_t horizon, const double updateInterval);

private:
   inline void addVehicle(const int id, const CarState & carState) { m_vehicles.emplace_back(Vehicle(id, carState)); }
   int m_nLanes;
   Lane m_lane;

   std::vector<Vehicle> m_vehicles;
};

#endif // ENVIRONMENT_HPP
