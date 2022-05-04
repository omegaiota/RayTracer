#ifndef PROJ6850_STATICSCENE_SCENE_H
#define PROJ6850_STATICSCENE_SCENE_H

#include "PROJ6850/PROJ6850.h"
#include "primitive.h"

#include <vector>
#include <set>

namespace PROJ6850 {
namespace StaticScene {


/**
 * A node in the BVH accelerator aggregate.
 * The accelerator uses a "flat tree" structure where all the primitives are
 * stored in one vector. A node in the data structure stores only the starting
 * index and the number of primitives in the node and uses this information to
 * index into the primitive vector for actual data. In this implementation all
 * primitives (index + range) are stored on leaf nodes. A leaf node has no child
 * node and its range should be no greater than the maximum leaf size used when
 * constructing the BVH.
 */
    struct AccelNode {
        AccelNode(BBox bb, size_t start, size_t range) // flat tree representaation for BVH
                : bb(bb), start(start), range(range), l(NULL), r(NULL) {} // set representation for kd-tree

        AccelNode(BBox bb, std::set<int>& indices)
                : bb(bb), start(0), range(0), indices(indices), l(NULL), r(NULL) {}

        inline bool isLeaf() const { return l == NULL && r == NULL; }

        BBox bb;       ///< bounding box of the node
        size_t start;  ///< start index into the primitive list
        size_t range;  ///< range of index into the primitive list

        std::set<int> indices; // index into the primitive list
        AccelNode *l;    ///< left child node
        AccelNode *r;    ///< right child node

    };

    struct TreeStat {
        int totalNodes;
        int maxLevel;
        int totalLeafNodes;
        int totalLeafTriangles;
    };

    struct RenderingStat {
        int totalVisitedNodes;
        int totalRayTriangleTest;
    };

/**
 * Interface for objects in the scene.
 */
class SceneObject {
 public:
  /**
   * Get all the primitives in the scene object.
   * \return a vector of all the primitives in the scene object
   */
  virtual std::vector<Primitive*> get_primitives() const = 0;

  /**
   * Get the surface BSDF of the object's surface.
   * \return the BSDF of the objects's surface
   */
  virtual BSDF* get_bsdf() const = 0;
};

/**
 * Interface for lights in the scene.
 */
class SceneLight {
 public:
  virtual Spectrum sample_L(const Vector3D& p, Vector3D* wi, float* distToLight,
                            float* pdf) const = 0;
  virtual bool is_delta_light() const = 0;
};

/**
 * Represents a scene in a raytracer-friendly format. To speed up raytracing,
 * all data is already transformed to world space.
 */
struct Scene {
  Scene(const std::vector<SceneObject*>& objects,
        const std::vector<SceneLight*>& lights)
      : objects(objects), lights(lights) {}

  // kept to make sure they don't get deleted, in case the
  //  primitives depend on them (e.g. Mesh Triangles).
  std::vector<SceneObject*> objects;

  // for sake of consistency of the scene object Interface
  std::vector<SceneLight*> lights;

  // TODO (sky) :
  // Adding object with emission BSDFs as mesh lights and sphere lights so
  // that light sampling configurations also applies to mesh lights.
  //  for (SceneObject *obj : objects) {
  //    if (obj->get_material().emit != Spectrum()) {
  //
  //      // mesh light
  //      if (dynamic_cast<Mesh*>(obj)) {
  //        staticLights.push_back()
  //      }
  //
  //      // sphere light
  //      if (dynamic_cast<Sphere*>(obj)) {
  //
  //      }
  //  }
};

}  // namespace StaticScene
}  // namespace PROJ6850

#endif  // PROJ6850_STATICSCENE_SCENE_H
