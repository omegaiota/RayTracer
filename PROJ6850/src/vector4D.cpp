#include "vector4D.h"

namespace PROJ6850 {

  std::ostream& operator<<( std::ostream& os, const Vector4D& v ) {
    os << "(" << v.x << "," << v.y << "," << v.z << "," << v.w << ")";
    return os;
  }

  Vector3D Vector4D::to3D() {
    return Vector3D(x, y, z);
  }

  Vector3D Vector4D::projectTo3D() {
    double invW = 1.0 / w;
    return Vector3D(x * invW, y * invW, z * invW);
  }

} // namespace PROJ6850
