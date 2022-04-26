#include <cassert>
#include "environment_light.h"

namespace PROJ6850 {
namespace StaticScene {

    EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
            : envMap(envMap) {

      std::vector<float> flux, p_theta, p_theta_phi;
      size_t w = envMap->w, h = envMap->h;
      flux.resize(w * h);
      p_theta.resize(h);
      p_theta_phi.resize(w * h);
      p_cdf_theta.resize(h);
      p_cdf_theta_phi.resize(w * h);
      float sum = 0.f;
      // weight illumination by sin
      for (int j = 0; j < h; j++) {
        float theta =  ((float) j + 0.5f) / ((float) h) * (float) PI;
        float sin = sinf(theta);
        for (int i = 0; i < w; i++) {
          float flux_v = clamp(envMap->data[i + j * envMap->w].illum(), 0.f, 1.f)  * sin;
          flux[i + j * envMap->w] = flux_v;
          sum += flux_v;
        }
      }

      // normalize
      float normFactor = 1.0f / sum;
      for (size_t i = 0; i < w * h; i++) {
        flux[i] *= normFactor;
      }

      // compute marginal marginal probability distribution
      // for selecting a value from each row of pixels, and
      //for any pixel, compute the conditional probability
      p_theta_phi = flux;
      float p_theta_sum = 0.f;
      for (int j = 0; j < h; j++) {
        float sum = 0.f;
        for (int i = 0; i < w; i++) {
          sum += flux[i + j * envMap->w];
        }
        //compute marginal marginal probability distribution
        // for selecting a value from each row of pixels
        p_theta[j] = sum;

        //Given for any pixel, compute the conditional probability
        for (int i = 0; i < w; i++) {
          p_theta_phi[i + j * envMap->w] = flux[i + j * envMap->w] / sum;
        }
      }



      // compute CDF
      p_theta_sum = 0.f;
      for (int j = 0; j < h; j++) {
        p_theta_sum += p_theta[j];
        p_cdf_theta[j] = p_theta_sum;

        float p_theta_phi_sum = 0.f;
        for (int i = 0; i < w; i++) {
          p_theta_phi_sum += p_theta_phi[i + j * envMap->w];
          p_cdf_theta_phi[i + j * envMap->w] = p_theta_phi_sum;
        }


      }
    }


    Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                        float* distToLight, float* pdf) const {
  // Uniform Sampling
//      *pdf = 1.0f / (4.0f * (float) PI);
//  double Xi1 = ((double)(std::rand()) / RAND_MAX) * 2.0 - 1.0;
//  double Xi2 = (double)(std::rand()) / RAND_MAX;
//
//  double theta = acos(Xi1);
//  if (theta < 0)
//    theta = theta + PI;
//  double phi = 2.0 * PI * Xi2;
//  *pdf = 1.0f / (float)  (4.0f * PI);
//  *distToLight = std::numeric_limits<float>::infinity();
//
//  double x = (phi / (2.0 * PI)) * ((double) envMap->w), y = theta / PI * ((double) envMap ->h);

      //importance sampling
      float Xi1 = ((float)(std::rand()) / RAND_MAX);
      float Xi2 = ((float)(std::rand()) / RAND_MAX);
      size_t i = 0, j = 0;
      for (j = 0; j < envMap->h; j++) {
        if (p_cdf_theta[j] >= Xi1)
          break;
      }

      for (i = 0; i < envMap->w; i++) {
        if (p_cdf_theta_phi[i + j * envMap->w] >= Xi2)
          break;
      }

      float theta = ((j + 0.5f) / (float) envMap->h) * (float) PI , phi = ((i + 0.5f) / (float) envMap->w) * ((float) PI) * 2.0f;
      *pdf = envMap->data[ i + j * envMap->w].illum() * (envMap->w * envMap->h / (4.0f * (float) PI * (float) PI * sin(theta)));

      //shared code
      double xs = sinf(theta) * cosf(phi);
      double ys = sinf(theta) * sinf(phi);
      double zs = cosf(theta);
      *wi = Vector3D(xs, ys, zs);
      *distToLight = std::numeric_limits<float>::infinity();
      return bilinear_interpolate_coor(i, j);
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  Vector3D dir = r.d;
  dir.normalize();
  double xx = dir.x, yy = dir.y, zz = dir.z;
  double theta = acos(yy), phi = atan2(xx, -1 * zz) + PI;
  if (phi < 0)
    phi += 2.0f * PI;


  double x = (phi / (2.0 * PI)) * ((double) envMap->w), y =theta / PI * ((double) envMap ->h);
  return bilinear_interpolate_coor(x, y);

}

    Spectrum EnvironmentLight::bilinear_interpolate_coor(double x, double y) const {
      //bilinear sampling
      Spectrum ul, ur, ll, lr;
      int x_min = std::max((int) x, 0) ;
      int y_min = std::max((int) y, 0);
      int x_max = std::min(x_min + 1, (int) envMap->w - 1), y_max = std::min(y_min + 1, (int) envMap->h - 1);
      double u = x - (double) x_min, v = y - (double) y_min;
      ul = envMap->data[((x_min) + y_max * envMap->w)];
      ur = envMap->data[((x_max) + y_max * envMap->w)];
      ll = envMap->data[((x_min) + y_min * envMap->w)];
      lr = envMap->data[((x_max) + y_min * envMap->w)];
      Spectrum interpolated = (1 - u) * ((1 - v) * ll + v * ul) +
                              u * ((1 - v) * lr + v * ur);
      return interpolated;
    }

}  // namespace StaticScene
}  // namespace PROJ6850
