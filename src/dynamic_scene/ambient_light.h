#ifndef PROJ6850_DYNAMICSCENE_AMBIENTLIGHT_H
#define PROJ6850_DYNAMICSCENE_AMBIENTLIGHT_H

#include "scene.h"
#include "../static_scene/light.h"

namespace PROJ6850 {
namespace DynamicScene {

class AmbientLight : public SceneLight {
 public:
  AmbientLight(const Collada::LightInfo& light_info) {
    this->spectrum = light_info.spectrum;
  }

  StaticScene::SceneLight* get_static_light() const {
    StaticScene::InfiniteHemisphereLight* l =
        new StaticScene::InfiniteHemisphereLight(spectrum);
    return l;
  }

 private:
  Spectrum spectrum;
};

}  // namespace DynamicScene
}  // namespace PROJ6850

#endif  // PROJ6850_DYNAMICSCENE_AMBIENTLIGHT_H
