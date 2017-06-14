/*
 *
 */

#include <vector>

namespace Pathfinder
{
  class Position
  {
    public:
      Position (double x, double y);

    private:
      double _x;
      double _y;
  };

  class MapObject
  {
    public:
      MapObject ();

      const std::vector<Position>& getPolygon () const;

    private:
      std::vector<Position> _poly;
  };

  class Map
  {
    public:
      Map ();
  };
}
