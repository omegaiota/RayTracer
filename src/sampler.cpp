#include "sampler.h"

namespace PROJ6850 {

// Uniform Sampler2D Implementation //

    Vector2D UniformGridSampler2D::get_sample() const {
      // Implement uniform 2D grid sampler
      return Vector2D((double)(std::rand()) / (double) RAND_MAX,
                      (double)(std::rand()) / (double) RAND_MAX);
    }

// Uniform Hemisphere Sampler3D Implementation //
    Vector3D UniformHemisphereSampler3D::get_sample() const {
      float Xi1 = (float)(std::rand()) / RAND_MAX;
      float Xi2 = (float)(std::rand()) / RAND_MAX;

      float theta = acosf(Xi1);
      float phi = 2.0f * (float) PI * Xi2;

      float xs = sinf(theta) * cosf(phi);
      float ys = sinf(theta) * sinf(phi);
      float zs = cosf(theta);

      return Vector3D(xs, ys, zs);
    }

    Vector3D CosineWeightedHemisphereSampler3D::get_sample() const {
      float f;
      return get_sample(&f);
    }

    Vector3D CosineWeightedHemisphereSampler3D::get_sample(float *pdf) const {
      // first uniformly sample in a unit disk, then project on to the surface of the hemisphere
      float Xi1 = (float)(std::rand()) / RAND_MAX;
      float Xi2 = (float)(std::rand()) / RAND_MAX;
      float r = sqrt(Xi1), phi = 2.0f * (float) PI * Xi2;
      float x = r * cosf(phi), y = r * sinf(phi), z = sqrtf(std::max(0.f, 1.f - x * x - y * y ));
      float theta = asinf(r);
//      *pdf = cosf(theta) / (float) PI;
      *pdf = 1 / (float) PI;
      return Vector3D(x, y, z).unit();
    }

}  // namespace PROJ6850