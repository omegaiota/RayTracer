#ifndef PROJ6850_DYNAMICSCENE_ENVIRONMENTLIGHT_H
#define PROJ6850_DYNAMICSCENE_ENVIRONMENTLIGHT_H

#include "scene.h"
#include "../image.h"
#include "../static_scene/light.h"

namespace PROJ6850 {
namespace DynamicScene {

class EnvironmentLight : public SceneLight {
 public:
  EnvironmentLight(HDRImageBuffer* envmap) : envmap(envmap) {}

  StaticScene::SceneLight* get_static_light() const {
    StaticScene::EnvironmentLight* l =
        new StaticScene::EnvironmentLight(envmap);
    return l;
  }

 private:
  HDRImageBuffer* envmap;
};

}  // namespace DynamicScene
}  // namespace PROJ6850

#endif  // PROJ6850_DYNAMICSCENE_ENVIRONMENTLIGHT_H
