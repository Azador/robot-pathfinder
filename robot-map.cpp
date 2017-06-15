/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-map.h"

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

  MapObject::MapObject ()
  : _poly ()
  {
  }

  const std::vector<Position>& MapObject::getPolygon () const
  {
    return _poly;
  }

  bool MapObject::isClosed () const
  {
    if (_poly.size () < 2)
      return false;

    return _poly.front () == _poly.back ();
  }

  bool MapObject::isEmpty () const
  {
    return _poly.empty ();
  }

  bool MapObject::join (const MapObject & other, double max_dist)
  {
    if (other.isEmpty ())
      return false;

    if (isEmpty ())
      {
        _poly = other._poly;
        return true;
      }

    std::optional<std::pair<double, uint32_t> > best_dist = findClosestPosition (other._poly[0]);
    uint32_t best_idx = 0;
    for (uint32_t i=1; i < other._poly.size (); ++i)
      {
        std::optional<std::pair<double, uint32_t> > dist = findClosestPosition (other._poly[i]);
        if (!dist.has_value ())
          continue;

        if (!best_dist.has_value () || dist->first < best_dist->first)
          {
            best_dist = dist;
            best_idx = i;
          }
      }

    if (!best_dist.has_value () || best_dist->first > max_dist)
      return false;

    // ToDo: Code for merging the polygons is missing.

    return false;
  }

  std::optional<std::pair<double, uint32_t> >
  MapObject::findClosestPosition (const Position & pos) const
  {
    std::optional<std::pair<double, uint32_t> > found;
    if (_poly.empty ())
      return found;

    if (_poly.size () == 1)
      {
        found = std::make_pair (_poly[0].distance (pos), 0);
        return found;
      }

    found = std::make_pair (LineSegment (_poly[0], _poly[1]).distance (pos), 0);
    for (uint32_t i=2; i < _poly.size (); ++i)
      {
        LineSegment lseg (_poly[i-1], _poly[i]);
        double dist = lseg.distance (pos);

        if (dist < found->first)
          found = std::make_pair (dist, i-1);
      }

    return found;
  }

  Map::Map ()
  : _objects ()
  {
  }
}
