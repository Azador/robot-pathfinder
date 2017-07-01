/*
 *
 */

#include <vector>
#include <cstdint>

#include "robot-geometry.h"

namespace Pathfinder
{
  class MapObject
  {
    public:
      MapObject (double min_point_distance);

      const std::vector<Position,Eigen::aligned_allocator<Position>>& getPolygon () const;
      bool isClosed () const;
      bool isEmpty () const;
      void appendPoint (const Position & point);
      void clear ();
      bool join (const MapObject & other, double max_dist);
      bool addPoint (const Position & point, double max_dist);
      void smooth (double max_deviation, uint32_t filter_size);
      void makeEquidistant (double max_dist, uint32_t min_points, double max_deviation);
      void convexHull ();

      struct FindResult
      {
          double distance;
          uint32_t point_index;
          double   fraction_to_next_point;
      };
      std::optional<FindResult> findClosestPosition (const Position & pos) const;

    private:
      double _min_point_distance;
      std::vector<Position,Eigen::aligned_allocator<Position>> _poly;
  };

  class Map
  {
    public:
      Map ();

      void addObject (const MapObject & obj);
      void addObject (MapObject && obj);
      const std::vector<MapObject> & getObjects () const;

    private:
      std::vector<MapObject> _objects;
  };
}
