#ifndef PROJ6850_INTERSECT_H
#define PROJ6850_INTERSECT_H

#include <vector>

#include "PROJ6850/vector3D.h"
#include "PROJ6850/spectrum.h"
#include "PROJ6850/misc.h"

#include "bsdf.h"

namespace PROJ6850 {
    namespace StaticScene {

        class Primitive;

/**
 * A record of an intersection point which includes the time of intersection
 * and other information needed for shading
 */
        struct Intersection {
            Intersection() : t(INF_D), primitive(NULL), bsdf(NULL) {}

            double t;  ///< time of intersection

            const Primitive* primitive;  ///< the primitive intersected

            Vector3D n;  ///< normal at point of intersection

            BSDF* bsdf;  ///< BSDF of the surface at point of intersection

        };

    }  // namespace StaticScene
}  // namespace PROJ6850

#endif  // PROJ6850_INTERSECT_H