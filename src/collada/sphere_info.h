#ifndef PROJ6850_COLLADA_SPHEREINFO_H
#define PROJ6850_COLLADA_SPHEREINFO_H

#include "collada_info.h"
#include "material_info.h"

namespace PROJ6850 {
namespace Collada {

struct SphereInfo : Instance {
  float radius;            ///< radius
  MaterialInfo* material;  ///< material of the sphere
};                         // struct Sphere

std::ostream& operator<<(std::ostream& os, const SphereInfo& sphere);

}  // namespace Collada
}  // namespace PROJ6850

#endif  // PROJ6850_COLLADA_SPHEREINFO_H
