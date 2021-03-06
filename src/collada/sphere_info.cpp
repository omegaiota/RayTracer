#include "sphere_info.h"

using namespace std;

namespace PROJ6850 {
namespace Collada {

std::ostream& operator<<(std::ostream& os, const SphereInfo& sphere) {
  return os << "Sphere: " << sphere.name << " (id:" << sphere.id << ") ["
            << " radius=" << sphere.radius << " ]";
}

}  // namespace Collada
}  // namespace PROJ6850
