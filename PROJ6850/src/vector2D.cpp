#include "vector2D.h"

namespace PROJ6850 {

  std::ostream& operator<<( std::ostream& os, const Vector2D& v ) {
    os << "(" << v.x << "," << v.y << ")";
    return os;
  }

} // namespace PROJ6850
