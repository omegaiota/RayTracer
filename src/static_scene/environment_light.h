#ifndef PROJ6850_STATICSCENE_ENVIRONMENTLIGHT_H
#define PROJ6850_STATICSCENE_ENVIRONMENTLIGHT_H

#include "../sampler.h"
#include "../image.h"
#include "scene.h"

namespace PROJ6850 {
    namespace StaticScene {

// An environment light can be thought of as an infinitely big sphere centered
// around your scene, radiating light on the scene in a pattern matching some
// image. This is commonly used for low-cost renderings of complex backrounds or
// environments (fancy church, overgrown forest, etc) that would be difficult to
// model in the scene.
        class EnvironmentLight : public SceneLight {
        public:
            EnvironmentLight(const HDRImageBuffer* envMap);
            /**
             * In addition to the work done by sample_dir, this function also has to
             * choose a ray direction to sample along. You should initially use a uniform
             * randomly distributed direction, but later you'll be required to implement
             * this with importance sampling. The requirements are:
             * - Sample within a pixel with probability proportional to the total power
             *   going through that pixel (so proportional to the pixel's area when
             *   projected onto the sphere, multiplied by the sum of the components of its
             *   Spectrum). This is a discrete probability distribution, so there's no
             *   formula.
             * - Don't take linear time to generate a single sample! You'll be calling
             *   this a LOT; it should be fast.
             */
            Spectrum sample_L(const Vector3D& p, Vector3D* wi, float* distToLight,
                              float* pdf) const;
            bool is_delta_light() const { return false; }
            /**
             * Returns the color found on the environment map by travelling in a specific
             * direction. This entails:
             * - Converting the 3-d ray direction into a 2-d image coordinate.
             * - Performing bilinear interpolation to determine the color at the given
             *   coordinate.
             * - Handling the edge cases correctly (what happens if you wrap around the
             *   environment map horizontally? What about vertically?).
             */
            Spectrum sample_dir(const Ray& r) const;



        private:
            const HDRImageBuffer* envMap;
            std::vector<float> p_cdf_theta;
            std::vector<float> p_cdf_theta_phi;

            /**
             * Bilinear interpolate the pixel in the environment map at (x,y)
             * @param x
             * @param y
             * @return
             */
            Spectrum bilinear_interpolate_coor(double x, double y) const;
        };  // class EnvironmentLight

    }  // namespace StaticScene
}  // namespace PROJ6850

#endif  // PROJ6850_STATICSCENE_ENVIRONMENTLIGHT_H