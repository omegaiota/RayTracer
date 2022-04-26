#ifndef PROJ6850_DYNAMICSCENE_MATERIAL_H
#define PROJ6850_DYNAMICSCENE_MATERIAL_H

#include "scene.h"
#include "../collada/material_info.h"

#include <iostream>

using std::cout;
using std::endl;

namespace PROJ6850 {
namespace DynamicScene {

class Material {
 public:
  Material() {
    ambient = Color(0, 0, 0, 0);
    diffuse = Color(0, 0, 0, 0);
    specular = Color(0, 0, 0, 0);
    shininess = 0;
  }

  Material(const Collada::MaterialInfo& info)
      : ambient(info.Ca),
        diffuse(info.Cd),
        specular(info.Cs),
        shininess(info.Ns) {}

  void set_material_properties() const {
    glBindTexture(GL_TEXTURE_2D, 0);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, &ambient.r);
    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, &diffuse.r);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, &specular.r);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
  }

 private:
  Color ambient;   ///< Color - ambient
  Color diffuse;   ///< Color - diffuse
  Color specular;  ///< Color - specular

  float shininess;  ///< Numerical - shininess
};

}  // namespace DynamicScene
}  // namespace PROJ6850

#endif  // PROJ6850_DYNAMICSCENE_MATERIAL_H
