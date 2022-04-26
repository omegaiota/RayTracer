#ifndef PROJ6850_COLLADA_CAMERAINFO_H
#define PROJ6850_COLLADA_CAMERAINFO_H

#include "collada_info.h"

namespace PROJ6850 {
namespace Collada {

/*
  Note that hFov_ and vFov_ are expected to be in DEGREES.
*/
struct CameraInfo : public Instance {
  Vector3D view_dir;
  Vector3D up_dir;

  float hFov, vFov, nClip, fClip;
};

std::ostream& operator<<(std::ostream& os, const CameraInfo& camera);

}  // namespace Collada
}  // namespace PROJ6850

#endif  // PROJ6850_COLLADA_CAMERAINFO_H
