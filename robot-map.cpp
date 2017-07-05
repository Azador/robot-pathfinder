/*
 *
 */

#include <cmath>
#include <limits>

#include "robot-map.h"

namespace Pathfinder
{
  /* Creating an empty object where two distinct points should be at least min_point_distance appart.
   */
  MapObject::MapObject (double min_point_distance)
  : _min_point_distance (min_point_distance),
    _poly ()
  {
  }

  /* Get the points of the object.
   *
   * If closed, the last point is at the same position as the first point.
   */
  const std::vector<Position,Eigen::aligned_allocator<Position>>& MapObject::getPolygon () const
  {
    return _poly;
  }

  /* Is the object closed and is not empty.
   *
   * An object is closed if it ends at the equal position as it starts.
   * Making at least two points necessary to have a closed object.
   */
  bool MapObject::isClosed () const
  {
    if (_poly.size () < 2)
      return false;

    return _poly.front () == _poly.back ();
  }

  /* Is the object empty (has not a single point) */
  bool MapObject::isEmpty () const
  {
    return _poly.empty ();
  }

  /* Append a single point as the last point.
   *
   * The closed state will be kept, min_point_distance will not be checked.
   */
  void MapObject::appendPoint (const Position & point)
  {
    if (isClosed ())
      {
	_poly.back () = point;
	_poly.push_back (_poly[0]);
      }
    else
      _poly.push_back (point);
  }

  /* Make the object closed or open.
   *
   * Making this object closed will copy the first point at the end, makeing it open, will remove
   * the last point. Changes will only happen, if it is not already in the desired state.
   */
  void MapObject::setClosed (bool closed)
  {
    if (isClosed () == closed)
      return;

    if (closed)
      _poly.push_back (_poly[0]);
    else
      _poly.pop_back ();
  }

  /* Clear the object by removing all points of it.
   */
  void MapObject::clear ()
  {
    _poly.clear ();
  }

  /* Join two MapObjects.
   *
   * The points should be in similar distances on both objects, otherwise this algorithm
   * might not work correctly.
   *
   * The objects must have positions, where they are not further than max_dist away.
   * There must be no splits in the resulting polygon.
   *
   * The resulting object should be smoothed and made equidistant again.
   */
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

    if (isClosed () || other.isClosed ())
    {
    	// ToDo: Check if the objects completely match and then join them to get more observations for the polygon
    	return false;
    }

    // At least two points in both objects
    // Find the first point of other, which is close enough to this.
    std::optional<MapObject::FindResult> dist;
    uint32_t idx = 0;

    for (uint32_t i=0; i < other._poly.size (); ++i)
      {
        dist = findClosestPosition (other._poly[i]);
        if (!dist.has_value ())
          continue;

        if (dist->distance <= max_dist)
          {
            idx = i;
            break;
          }
      }

    if (!dist.has_value () || dist->distance > max_dist)
      return false;

    uint32_t first_idx = idx;

    // We should not have splits of the line.
    // Check if the next three points before the found point are closest to the same
    // point on this.
    for (uint32_t i=idx; i > 0 && i > idx-3; --i)
      {
        std::optional<MapObject::FindResult> dist2
        = findClosestPosition (other._poly[i-1]);

        if (dist2->point_index != dist->point_index)
          {
            // This seems to be a split
            return false;
          }
      }

    // The next points have to be either close to this or one of the polygons have to end.
    for (uint32_t i=idx+1; i < other._poly.size (); ++i)
      {
        std::optional<MapObject::FindResult> dist2
        = findClosestPosition (other._poly[i]);

        if (dist2->distance > max_dist)
          break;

        dist = dist2;
        idx = i;
      }

    // Check if the next three points after the last found point are closest to the same
    // point on this.
    for (uint32_t i=idx; i < other._poly.size () && i < idx+3; --i)
      {
        std::optional<MapObject::FindResult> dist2
        = findClosestPosition (other._poly[i]);

        if (dist2->point_index != dist->point_index)
          {
            // This seems to be a split
            return false;
          }
      }

    // Now check all remaining points, they should not be near this again.
    // Only exception: This and other together create a closed polygon.
    bool found_circle = false;
    for (uint32_t i=idx+3; i < other._poly.size (); ++i)
      {
        std::optional<MapObject::FindResult> dist2
        = findClosestPosition (other._poly[i]);

        if (dist2->distance < max_dist)
          {
            found_circle = false;
            idx = i;
            dist = dist2;
            break;
          }
      }

    if (found_circle)
      {
        // This is the _second_ matching part of the polygons.

        // If it is not the first or last point of this, it is the wrong point.
        if (dist->point_index != 0 && dist->point_index != _poly.size () - 1)
          return false;

        // if the first matching part didn't start with the first point of other,
        // it was the wrong point.
        if (first_idx != 0)
          return false;

        // Follow the second matching part to the end.
        for (uint32_t i=idx+1; i < other._poly.size (); ++i)
          {
            std::optional<MapObject::FindResult> dist2
            = findClosestPosition (other._poly[i]);

            if (dist2->distance > max_dist)
              {
                // It must not leave this any more.
                return false;
              }
          }
      }

    // Polygons have no split and can be merged.
    if (first_idx > 0)
    	_poly.insert (_poly.begin (), other._poly.begin (), other._poly.begin () + first_idx);

    for (uint32_t i=first_idx; i < other._poly.size (); ++i)
    {
        std::optional<MapObject::FindResult> dist2
        = findClosestPosition (other._poly[i]);

        if (dist2->distance < max_dist)
        {
        	dist = dist2;
        	addPoint (other._poly[i], max_dist);
        }
        else
        {
        	if (dist->fraction_to_next_point == 1.0)
        		_poly.insert (_poly.begin () + dist->point_index + 2, other._poly[i]);
        	else
        		_poly.insert (_poly.begin () + dist->point_index + 1, other._poly[i]);
        }
    }

    if (found_circle)
    	_poly.push_back (_poly[0]);

    return true;
  }

  /* Add a point to the object at the right position.
   *
   * The point will be added if it is at least _min_point_distance away from the next point
   * in this object and at most max_dist away from the curve of this object.
   *
   * It will be added at the right position.
   *
   * Returns if the point was close enough to the curve.
   */
  bool MapObject::addPoint (const Position & point, double max_dist)
  {
    if (_poly.empty ())
      {
	// first point -> just add it
	_poly.push_back (point);
	return true;
      }

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
          _poly.push_back (point);
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
    if (point.distance (_poly[dist->point_index]) >= _min_point_distance)
    {
    	if (_poly[dist->point_index-1].distance (point) < _poly[dist->point_index+1].distance(point))
    		_poly.insert (_poly.begin () + dist->point_index, point);
    	else
    		_poly.insert (_poly.begin () + dist->point_index + 1, point);
    }

    return true;
  }

  /* Smooth the polygon by moving points if they are not further away than max_deviation
   * from a polynomial fitting curve (degree 2) of the next filter_size surrounding points
   * (min. 2) to each side, including the point in question.
   */
  void MapObject::smooth (double max_deviation, uint32_t filter_size)
  {
    bool closed = isClosed ();

    if (filter_size < 2)
      filter_size = 2;

    uint32_t min_points = filter_size * 2 + 1;
    if (closed)
      ++min_points;

    if (_poly.size () < min_points)
      return;

    std::vector<Position,Eigen::aligned_allocator<Position>> filter_array (filter_size * 2 + 1);
    std::vector<Position,Eigen::aligned_allocator<Position>> new_poly (_poly.size ());
    PolynomCurve<2> poly_curve;
    std::optional<double> residual;

    for (uint32_t i = closed ? 1 : 0; i < _poly.size (); ++i)
      {
        bool need_adjust = true;

        if (i < filter_size || i + filter_size >= _poly.size ())
          {
            if (closed)
              {
                if (i < filter_size)
                  {
                    uint32_t d = filter_size - i;
                    uint32_t k = 0;
                    for (uint32_t j=_poly.size () - d; j < _poly.size (); ++j, ++k)
                      filter_array[k] = _poly[j];

                    // Leave out index 0, that's the same point as the last one in _poly (isClosed)
                    d = filter_size*2 + 1 - d;
                    for (uint32_t j=1; j <= d; ++j, ++k)
                      filter_array[k] = _poly[j];
                  }
                else
                  {
                    uint32_t k = 0;
                    for (uint32_t j=i-filter_size; j < _poly.size (); ++j, ++k)
                      filter_array[k] = _poly[j];

                    // Leave out index 0, that's the same point as the last one in _poly (isClosed)
                    uint32_t d = (i + filter_size + 1) - _poly.size ();
                    for (uint32_t j=1; j <= d; ++j, ++k)
                      filter_array[k] = _poly[j];
                  }
              }
            else
              {
                if (i == 0)
                  for (uint32_t j=0; j <= filter_size*2; ++j)
                    filter_array[j] = _poly[j];
                else if (i + filter_size == _poly.size ())
                  for (uint32_t j=_poly.size () - filter_size*2 - 1; j < _poly.size (); ++j)
                    filter_array[j - (_poly.size () - filter_size*2 - 1)] = _poly[j];
                else
                  need_adjust = false;
              }
          }
        else
          {
            for (uint32_t j=i-filter_size; j <= i+filter_size; ++j)
              filter_array[j-(i-filter_size)] = _poly[j];
          }

        if (need_adjust)
          residual = poly_curve.adjust (filter_array);

        if (residual.has_value () && *residual <= max_deviation)
          {
            Position pnew = poly_curve.projectOnCurve (_poly[i]);

            if (pnew.distance (_poly[i]) <= max_deviation)
              new_poly[i] = pnew;
            else
              new_poly[i] = _poly[i];
          }
        else
          new_poly[i] = _poly[i];
      }

    if (closed)
      new_poly[0] = new_poly.back ();

    _poly.swap (new_poly);
  }

  /* Change the number of points, so that at least min_points points exist and these points
   * are preferably max_dist or less apart from each other.
   * But points only get a bigger distance than they actually have if the dropped or
   * moved point won't be more than max_deviation away from the the new resulting
   * polygon.
   */
  void MapObject::makeEquidistant (double max_dist, uint32_t min_points, double max_deviation)
  {
    // ToDo: Implementation missing
  }

  /* Change this MapObject to its convex hull.
   * If this is not closed, it will be closed after creating the convex hull by
   * connecting the end point to the start point.
   * Crossings will be removed.
   *
   * There are less than three distinct points, this method will do nothing.
   */
  void MapObject::convexHull ()
  {
    // less than four points are always convex (triangle).
    if (_poly.size () < 4)
      {
        if (_poly.size () == 3)
          _poly.push_back (_poly[0]); // Close the triangle
        return;
      }

    if (isClosed ())
      {
        // Closed: four points are three points...
        if (_poly.size () == 4)
          return;

        // Closed -> remove the last point to not have it doubled
        _poly.pop_back ();
      }

    Position prev = _poly[0];
    uint32_t prev_idx = 0;
    for (uint32_t i=1; i < _poly.size (); ++i)
      if (_poly[i].x () < prev.x ()
        || (_poly[i].x () == prev.x ()
          && _poly[i].y () < prev.y ()))
        {
          prev = _poly[i];
          prev_idx = i;
        }

    uint32_t first_idx = prev_idx;

    std::vector<Position,Eigen::aligned_allocator<Position>> hull;
    hull.push_back (prev);

    static double pi = std::acos (-1);
    uint32_t next_idx;

    do
      {
        next_idx = prev_idx==0 ? 1 : 0;
        Position next = _poly[next_idx];
        Eigen::Vector2d next_dir = next - prev;
        double next_a = std::atan2 (next_dir.y (), next_dir.x ());

        for (uint32_t i=0; i < _poly.size (); ++i)
          {
            if (i == prev_idx)
              continue;

            Eigen::Vector2d dir = _poly[i] - prev;
            double a = std::atan2 (dir.y (), dir.x ());
            double da = a - next_a;
            if (da < -pi)
              da += 2*pi;

            if (da > 0)
              {
                next_idx = i;
                next = _poly[i];
                next_dir = dir;
                next_a = a;
              }
          }

        hull.push_back (next);
        prev = next;
        prev_idx = next_idx;
      }
    while (next_idx != first_idx);

    _poly.swap (hull);
  }

  std::optional<MapObject::FindResult>
  MapObject::findClosestPosition (const Position & pos) const
  {
    std::cerr << "size: " << _poly.size () << std::endl;
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

  void Map::addObject (const MapObject & obj)
  {
    _objects.push_back (obj);
  }

  void Map::addObject (MapObject && obj)
  {
    _objects.push_back (obj);
  }

  const std::vector<MapObject> & Map::getObjects () const
  {
    return _objects;
  }
}
