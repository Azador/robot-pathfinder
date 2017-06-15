/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-map.h"

namespace Pathfinder
{
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
