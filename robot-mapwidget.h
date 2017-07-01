/*
 *
 */

#include <vector>
#include <cstdint>

#include <QGraphicsScene>

#include "robot-map.h"

namespace Pathfinder
{
  class MapScene : public QGraphicsScene
  {
    public:
      MapScene (QObject * parent, Map * map);

      void updateScene ();

    private:
      Map * _map;
  };
}
