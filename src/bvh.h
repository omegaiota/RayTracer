#ifndef PROJ6850_BVH_H
#define PROJ6850_BVH_H

#include "static_scene/scene.h"
#include "static_scene/aggregate.h"

#include <vector>

namespace PROJ6850 {
    namespace StaticScene {


/**
 * Bounding Volume Hierarchy for fast Ray - Primitive intersection.
 * Note that the BVHAccel is an Aggregate (A Primitive itself) that contains
 * all the primitives it was built from. Therefore once a BVHAccel Aggregate
 * is created, the original input primitives can be ignored from the scene
 * during ray intersection tests as they are contained in the aggregate.
 */
        class BVHAccel : public Aggregate {
        public:
            BVHAccel() {}

            /**
             * Parameterized Constructor.
             * Create BVH from a list of primitives. Note that the BVHAccel Aggregate
             * stores pointers to the primitives and thus the primitives need be kept
             * in memory for the aggregate to function properly.
             * \param primitives primitives to build from
             * \param max_leaf_size maximum number of primitives to be stored in leaves
             */
            BVHAccel(const std::vector<Primitive *> &primitives, size_t max_leaf_size = 4);

            /**
             * Destructor.
             * The destructor only destroys the Aggregate itself, the primitives that
             * it contains are left untouched.
             */
            ~BVHAccel();

            /**
             * Get the world space bounding box of the aggregate.
             * \return world space bounding box of the aggregate
             */
            BBox get_bbox() const {
              return  root->bb;
            }

            /**
             * Ray - Aggregate intersection.
             * Check if the given ray intersects with the aggregate (any primitive in
             * the aggregate), no intersection information is stored.
             * \param r ray to test intersection with
             * \return true if the given ray intersects with the aggregate,
                       false otherwise
             */
            bool intersect(const Ray &r, RenderingStat& renderingStat) const;
            bool intersect(const Ray &r) const;
            /**
             * Ray - Aggregate intersection 2.
             * Check if the given ray intersects with the aggregate (any primitive in
             * the aggregate). If so, the input intersection data is updated to contain
             * intersection information for the point of intersection. Note that the
             * intersected primitive entry in the intersection should be updated to
             * the actual primitive in the aggregate that the ray intersected with and
             * not the aggregate itself.
             * \param r ray to test intersection with
             * \param i address to store intersection info
             * \return true if the given ray intersects with the aggregate,
                       false otherwise
             */
            bool intersect(const Ray &r, Intersection *i, RenderingStat& renderingStat) const;
            bool intersect(const Ray &r, Intersection *i ) const;

            /**
             * Get BSDF of the surface material
             * Note that this does not make sense for the BVHAccel aggregate
             * because it does not have a surface material. Therefore this
             * should always return a null pointer.
             */
            BSDF *get_bsdf() const { return NULL; }

            /**
             * Get entry point (root) - used in visualizer
             */
            AccelNode *get_root() const { return root; }

            /**
             * Draw the BVH with OpenGL - used in visualizer
             */
            void draw(const Color &c) const {}

            /**
             * Draw the BVH outline with OpenGL - used in visualizer
             */
            void drawOutline(const Color &c) const {}

        private:
            AccelNode *root;  ///< root node of the BVH
            AccelNode *recursiveBuild(size_t start, size_t end, size_t &totalNodesBuild,
                                      std::vector<Primitive *> &orderedPrimitives,
                                      const std::vector<Primitive *> &originalPrimitives,
                                      size_t max_leaf_size, int level, TreeStat& treeStat); ///< helper function for recursively building BVH
            void traverse(const Ray &ray, AccelNode* currentNode, Intersection *isect, bool &hits, RenderingStat& renderingStat) const;
            void  recursiveDelete(AccelNode* node);

        };  // namespace StaticScene
    };
}// namespace PROJ6850

#endif  // PROJ6850_BVH_H
