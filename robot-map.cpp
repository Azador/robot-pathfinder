/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-map.h"

namespace Pathfinder
{
  MapObject::MapObject (double min_point_distance)
  : _min_point_distance (min_point_distance),
    _poly ()
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
      // other is empty, nothing to do
      return false;

    if (isEmpty ())
      {
        // this is empty, copy other...
        _poly = other._poly;
        return true;
      }

    if (_poly.size () == 1 && other._poly.size () == 1)
      {
        // Simple case: Two single points. Join them if closer than max_dist.
        if (_poly[0].distance (other._poly[0]) > max_dist)
          return false;

        _poly.push_back (other._poly[0]);
        return true;
      }

    if (other._poly.size () == 1)
      // Special case: The other object contains only a single point
      // use addPoint
      return addPoint (other._poly[0], max_dist);

    if (_poly.size () == 1)
      {
        // Special case: This objects contains only a single point
        // use addPoint
        MapObject o2 = other;
        if (!o2.addPoint (_poly[0], max_dist))
          return false;
        *this = o2;
        return true;
      }

    // At least two points in both objects

    std::optional<MapObject::FindResult> best_dist = findClosestPosition (other._poly[0]);
    uint32_t best_idx = 0;
    for (uint32_t i=1; i < other._poly.size (); ++i)
      {
        std::optional<MapObject::FindResult> dist = findClosestPosition (other._poly[i]);
        if (!dist.has_value ())
          continue;

        if (!best_dist.has_value () || dist->distance < best_dist->distance)
          {
            best_dist = dist;
            best_idx = i;
          }
      }

    if (!best_dist.has_value () || best_dist->distance > max_dist)
      return false;

    // ToDo: Code for merging the polygons is missing.

    return false;
  }

  bool MapObject::addPoint (const Position & point, double max_dist)
  {
    std::optional<MapObject::FindResult> dist = findClosestPosition (point);

    if (!dist.has_value () || dist->distance > max_dist)
      return false;

    if (dist->point_index == 0 && dist->fraction_to_next_point == 0.0)
      {
        // Before point 0 -> use as first point
        if (point.distance (_poly[0]) >= _min_point_distance)
          _poly.insert (_poly.begin (), point);
        return true;
      }

    if (dist->point_index == _poly.size () - 2 && dist->fraction_to_next_point == 1.0)
      {
        // After last point -> use as last point
        if (point.distance (_poly.back ()) >= _min_point_distance)
          _poly.insert (_poly.end (), point);
        return true;
      }

    if (dist->fraction_to_next_point > 0.0
        && dist->fraction_to_next_point < 1.0)
      {
        // Between points -> insert
        if (point.distance (_poly[dist->point_index]) >= _min_point_distance
            && point.distance (_poly[dist->point_index+1]) >= _min_point_distance)
          _poly.insert (_poly.begin () + dist->point_index + 1, point);
        return true;
      }

    // point is on at a point of poly
    if (dist->fraction_to_next_point  >= 1.0)
      dist->point_index++;

    // fraction == 0.0
    if (_poly[dist->point_index-1].distance (point) < _poly[dist->point_index+1].distance(point))
      {
        _poly.insert (_poly.begin () + dist->point_index, point);
      }
    else
      {
        _poly.insert (_poly.begin () + dist->point_index + 1, point);
      }

    return true;
  }

  std::optional<MapObject::FindResult>
  MapObject::findClosestPosition (const Position & pos) const
  {
    std::optional<MapObject::FindResult> found;
    if (_poly.empty ())
      return found;

    if (_poly.size () == 1)
      {
        found.emplace ();
        found->distance = _poly[0].distance (pos);
        found->point_index = 0;
        found->fraction_to_next_point = 0.0;
        return found;
      }

    found.emplace ();
    found->distance = LineSegment (_poly[0], _poly[1]).distance (pos, &found->fraction_to_next_point);
    found->point_index = 0;
    for (uint32_t i=2; i < _poly.size (); ++i)
      {
        LineSegment lseg (_poly[i-1], _poly[i]);
        double fraction_to_next_point = 0.0;
        double dist = lseg.distance (pos, &fraction_to_next_point);

        if (dist < found->distance)
          {
            found->distance = dist;
            found->point_index = i-1;
            found->fraction_to_next_point = fraction_to_next_point;
          }
        else if (dist == found->distance && fraction_to_next_point == 0.0)
          {
            found->distance = dist;
            found->point_index = i-1;
            found->fraction_to_next_point = fraction_to_next_point;
          }
      }

    return found;
  }

  Map::Map ()
  : _objects ()
  {
  }
}
