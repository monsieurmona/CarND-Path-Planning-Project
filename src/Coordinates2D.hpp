#ifndef COORDINATES2D_HPP
#define COORDINATES2D_HPP

#include <array>

#include "Coordinate2D.hpp"

template<size_t N>
class Coordinates2D
{
public:
   using Axis = std::array<double, N>;

   void simpleTrajectory(const Coordinate2D & position, const double intervall, const double rotationInRad)
   {
      for (size_t i = 0; i < N; ++i)
      {
         const Coordinate2D coordinate = position.move(intervall * i, rotationInRad);
         m_x[i] = coordinate.getX();
         m_y[i] = coordinate.getY();
      }
   }

   const Axis & getX() const {return m_x;}
   const Axis & getY() const {return m_y;}

   const Axis & getS() const {return m_x;}
   const Axis & getD() const {return m_y;}
private:
   Axis m_x; // respectivly s
   Axis m_y; // respectivly d
};

#endif // COORDINATES2D_HPP
