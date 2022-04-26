#ifndef PROJ6850_COLLADA_MATERIALINFO_H
#define PROJ6850_COLLADA_MATERIALINFO_H

#include "PROJ6850/color.h"
#include "collada_info.h"
#include "../bsdf.h"

namespace PROJ6850 {
namespace Collada {

struct MaterialInfo : public Instance {
  BSDF* bsdf;

  // Texture* tex; ///< texture

};  // struct Material

std::ostream& operator<<(std::ostream& os, const MaterialInfo& material);

}  // namespace Collada
}  // namespace PROJ6850

#endif  // PROJ6850_COLLADA_MATERIALINFO_H
