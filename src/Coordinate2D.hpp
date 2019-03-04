#ifndef COORDINATE2D_HPP
#define COORDINATE2D_HPP

#include <math.h>

class Coordinate2D
{
public:
   Coordinate2D();
   Coordinate2D(const double x, const double y) : m_x{x}, m_y{y} {}

   double getX() const {return m_x;}
   double getY() const {return m_y;}

   double getS() const {return m_x;}
   double getD() const {return m_y;}

   /**
    * @brief set coordinates
    * @param x x or s coordinate
    * @param y y or d coordinate
    */
   void set(const double x, const double y)
   {
      m_x = x;
      m_y = y;
   }

   Coordinate2D move(const double distance, const double rotateInRad) const
   {
      const double x = m_x + distance * cos(rotateInRad);
      const double y = m_y + distance * sin(rotateInRad);
      return Coordinate2D(x, y);
   }

private:
   double m_x = 0.0; // or s
   double m_y = 0.0; // or d
};

#endif // COORDINATE2D_HPP
