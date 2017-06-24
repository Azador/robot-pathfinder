/*
 *
 */

#include <vector>
#include <cstdint>

#include <qt5/QtWidgets/qgraphicsscene.h>

#include "robot-map.h"

namespace Pathfinder
{
  class MapScene : public QGraphicsScene
  {
    public:
      MapScene (QObject * parent, Map * map);

    private:
      Map * _map;
  };
}
