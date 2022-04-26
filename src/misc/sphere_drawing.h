#ifndef PROJ6850_UTIL_SPHEREDRAWING_H
#define PROJ6850_UTIL_SPHEREDRAWING_H

#include "PROJ6850/PROJ6850.h"

namespace PROJ6850 {
    namespace Misc {

/**
 * Draws a sphere with the given position and radius in opengl, using the
 * current modelview/projection matrices and the given color.
 */
        void draw_sphere_opengl(const Vector3D& p, double r, const Color& c);

/**
 * Draws a sphere with the given position and radius in opengl, using the
 * current modelview/projection matrices and color/material settings.
 */
        void draw_sphere_opengl(const Vector3D& p, double r);

    }  // namespace Misc
}  // namespace PROJ6850

#endif  // PROJ6850_UTIL_SPHEREDRAWING_H