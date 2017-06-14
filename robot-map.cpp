/*
 *
 */

#include "robot-map.h"

namespace Pathfinder
{
  Position::Position (double x, double y)
  : _x (x),
    _y (y)
  {
  }

  bool Position::operator== (const Position & other) const
  {
	  return other._x == _x && other._y == _y;
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

  Map::Map ()
  : _objects ()
  {
  }
}
