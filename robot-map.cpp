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

  MapObject::MapObject ()
  : _poly ()
  {
  }

  const std::vector<Position>& MapObject::getPolygon () const
  {
    return _poly;
  }

  Map::Map ()
  {
  }
}
