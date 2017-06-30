/*
 *
 */

#include <vector>
#include <cstdint>

#include <QtWidgets/qgraphicsscene.h>

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
