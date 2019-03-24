/** Author: Mario Lüder
 * Date: 2019-03-24
 */

#ifndef VEHICLE_HPP
#define VEHICLE_HPP

#include <vector>
#include <iostream>

#include "CarState.hpp"

class Vehicle
{
public:
   Vehicle(const int id, const CarState & state) : m_id(id), m_state(state) {}

   inline const CarState & getCarState() const { return m_state; }

private:
   int m_id;
   CarState m_state;
};

#endif // VEHICLE_HPP
