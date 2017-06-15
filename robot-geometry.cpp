/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-geometry.h"

namespace Pathfinder
{
  Position::Position ()
  : _x (),
    _y ()
  {
  }

  Position::Position (double x, double y)
  : _x (x),
    _y (y)
  {
  }

  double Position::distance (const Position & other) const
  {
    double dx = other._x - _x;
    double dy = other._y - _y;
    return sqrt (dx*dx + dy*dy);
  }

  bool Position::operator== (const Position & other) const
  {
	  return other._x == _x && other._y == _y;
  }

  Position Position::operator+ (const Position & other) const
  {
    Position result = *this;
    result += other;
    return result;
  }

  Position Position::operator- (const Position & other) const
  {
    Position result = *this;
    result -= other;
    return result;
  }

  Position Position::operator* (double factor) const
  {
    Position result = *this;
    result *= factor;
    return result;
  }

  Position & Position::operator+= (const Position & other)
  {
    _x += other._x;
    _y += other._y;
    return *this;
  }

  Position & Position::operator-= (const Position & other)
  {
    _x -= other._x;
    _y -= other._y;
    return *this;
  }

  Position & Position::operator*= (double factor)
  {
    _x *= factor;
    _y *= factor;
    return *this;
  }

  double Position::operator* (const Position & other) const
  {
    return _x * other._x + _y * other._y;
  }

  double Position::length2 () const
  {
    return _x*_x + _y*_y;
  }


  Line::Line (const Position & p1, const Position & p2)
  : _p1 (p1),
    _p2 (p2)
  {
  }

  Line::~Line ()
  {
  }

  Position Line::perpend (const Position & pos, double *t) const
  {
    Position dir = _p2 - _p1;
    Position r = pos - _p1;

    *t = (r*dir) / dir.length2 ();

    return _p1 + dir * *t;
  }

  double Line::distance (const Position & pos) const
  {
    double t;
    Position p = perpend (pos, &t);
    return p.distance (pos);
  }

  double Line::distance (const Position & pos, double *t) const
  {
    Position p = perpend (pos, t);
    return p.distance (pos);
  }

  const Position & Line::getPosition1 () const
  {
    return _p1;
  }

  const Position & Line::getPosition2 () const
  {
    return _p2;
  }

  Position Line::getDirection () const
  {
    return _p2 - _p1;
  }

  LineSegment::LineSegment (const Position & p1, const Position & p2)
  : Line (p1, p2)
  {
  }

  LineSegment::~LineSegment ()
  {
  }

  Position LineSegment::perpend (const Position & pos, double *t) const
  {
    Position result = Line::perpend (pos, t);

    if (*t < 0)
      {
        *t = 0;
        result = getPosition1 ();
        return result;
      }

    if (*t > 1)
      {
        *t = 1;
        result = getPosition2 ();
        return result;
      }

    return result;
  }
}
