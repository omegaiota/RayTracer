#include "sphere.h"

#include <cmath>
#include <cassert>

#include "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace PROJ6850 {
    namespace StaticScene {

        bool Sphere::test(const Ray& r, double& t1, double& t2) const {
          // Implement ray - sphere intersection test.
          // Return true if there are intersections and writing the
          // smaller of the two intersection times in t1 and the larger in t2.

          // convert ray to object space
          Vector3D o_objSpace = r.o - o, d = r.d;

          // solve (ox+tdx)^2 + (oy+tdy)^2 + (oz+tdz)^2 = r^2
          double  A = d.norm2(), B = 2.0 * dot(d, o_objSpace),
                  C = o_objSpace.norm2() - r2;
          double delta = B * B - 4.0 * A * C;
          if (delta < 0.0)
            return false;

          double rootDelta = sqrtf(delta);
            t1 = -0.5f * (B + rootDelta) / A;
            t2 = -0.5f * (B - rootDelta) / A;
          return t2 > r.min_t;
        }

        bool Sphere::intersect(const Ray& r) const {
          double t1 = 0, t2 = 0;
          if (test(r, t1, t2)) {
            return (isBetween(t1, r.min_t, r.max_t) || isBetween(t2, r.min_t, r.max_t));
          }
          return false;
        }

        bool Sphere::intersect(const Ray& r, Intersection* isect) const {
          // Implement ray - sphere intersection.
          // Note again that you might want to use the the Sphere::test helper here.
          // When an intersection takes place, the Intersection data should be updated
          // correspondingly.
          double t1 = 0, t2 = 0;
          if (test(r, t1, t2) && (isBetween(t1, r.min_t, r.max_t) || isBetween(t2, r.min_t, r.max_t))) {
            double t_hit = t1;
            if (!isBetween(t1, r.min_t, r.max_t)) {
              t_hit = t2;
              assert(isBetween(t2, r.min_t, r.max_t));
            }
            if (t_hit >= isect->t)
              return false;
            isect->t = t_hit;
            isect->bsdf = get_bsdf();
            isect->primitive = this;

            Vector3D hitPoint = r.o + t_hit * r.d;
            isect->n = normal(hitPoint);

            r.max_t = t_hit;
            return true;
          }
          return false;
        }

        void Sphere::draw(const Color& c) const { Misc::draw_sphere_opengl(o, r, c); }

        void Sphere::drawOutline(const Color& c) const {
          // Misc::draw_sphere_opengl(o, r, c);
        }

        bool Sphere::isBetween(double testNum, double min, double max) const {
          return ((testNum >= min) && (testNum <= max));
        }

    }  // namespace StaticScene
}  // namespace PROJ6850