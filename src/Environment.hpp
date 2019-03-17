#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include <vector>

#include "Track.hpp"
#include "Vehicle.hpp"

class Environment
{
public:
   Environment() {}

   void setEnvironment(const std::vector<std::vector<double>> & sensorFusion);
   void predict(const Track & track, const double_t horizon, const double updateInterval);

private:
   inline void addVehicle(const int id, const CarState & carState) { m_vehicles.emplace_back(Vehicle(id, carState)); }
   std::vector<Vehicle> m_vehicles;
};

#endif // ENVIRONMENT_HPP
