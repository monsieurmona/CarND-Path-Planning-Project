#ifndef COORDINATE2D_HPP
#define COORDINATE2D_HPP

#include <math.h>
#include <vector>

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

   void transformToCoordinateSystem(const Coordinate2D  & referenceCoordinates, const double referenceYaw)
   {
      const double shiftX = m_x - referenceCoordinates.m_x;
      const double shiftY = m_y - referenceCoordinates.m_y;

      const double cosRefYaw = cos(-referenceYaw);
      const double sinRefYaw = sin(-referenceYaw);

      m_x = shiftX * cosRefYaw - shiftY * sinRefYaw;
      m_y = shiftX * sinRefYaw + shiftY * cosRefYaw;
   }

   void inverseTransformToCoordinateSystem(const Coordinate2D & referenceCoordinates, const double referenceYaw)
   {
      const double shiftX = m_x;
      const double shiftY = m_y;

      const double cosRefYaw = cos(referenceYaw);
      const double sinRefYaw = sin(referenceYaw);

      m_x = shiftX * cosRefYaw - shiftY * sinRefYaw;
      m_y = shiftX * sinRefYaw + shiftY * cosRefYaw;

      m_x += referenceCoordinates.m_x;
      m_y += referenceCoordinates.m_y;
   }

   // Calculate distance between two points
   double distance(const Coordinate2D & otherCoordinate) const
   {
      const double x2 = otherCoordinate.m_x;
      const double y2 = otherCoordinate.m_y;
      const double x1 = m_x;
      const double y1 = m_y;
      return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
   }

   // Calculate distance to a point
   double distance() const
   {
      const double x1 = m_x;
      const double y1 = m_y;
      return sqrt((x1 * x1) + (y1 * y1));
   }

   double heading(const Coordinate2D & otherCoordinate) const
   {
      const double x1 = m_x;
      const double y1 = m_y;
      const double x2 = otherCoordinate.m_x;
      const double y2 = otherCoordinate.m_y;
      const double heading = atan2((y1-y2),(x1-x2));
      return heading;
   }

   Coordinate2D operator-(const Coordinate2D & rhs) const
   {
      return Coordinate2D(m_x - rhs.m_x, m_y - rhs.m_y);
   }

   Coordinate2D operator+(const Coordinate2D & rhs) const
   {
      return Coordinate2D(m_x + rhs.m_x, m_y + rhs.m_y);
   }

   // projection this onto n
   Coordinate2D projection(const Coordinate2D & n) const
   {
      const double x_x = m_x;
      const double x_y = m_y;
      const double n_x = n.m_x;
      const double n_y = n.m_y;
      const double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
      return Coordinate2D(proj_norm * n_x, proj_norm * n_y);
   }

private:
   double m_x = 0.0; // or s
   double m_y = 0.0; // or d
};

#endif // COORDINATE2D_HPP
