#include "bsdf.h"

#include <algorithm>
#include <iostream>
#include <utility>


using std::min;
using std::max;
using std::swap;

namespace PROJ6850 {

    void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {
      Vector3D z = Vector3D(n.x, n.y, n.z);
      Vector3D h = z;
      if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
        h.x = 1.0;
      else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
        h.y = 1.0;
      else
        h.z = 1.0;

      z.normalize();
      Vector3D y = cross(h, z);
      y.normalize();
      Vector3D x = cross(z, y);
      x.normalize();

      o2w[0] = x;
      o2w[1] = y;
      o2w[2] = z;
    }

// Diffuse BSDF //

    Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return albedo * (1.0 / PI);
    }

    Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // Implement DiffuseBSDF
      Vector3D sample = sampler.get_sample(pdf);
      *wi = sample;
      return albedo * (1.0 / PI);
    }

// Mirror BSDF //

    Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // Implement MirrorBSDF
      reflect(wo, wi);
      *pdf = 1.0f;
      return reflectance * (1.0f / abs(wo.z));
    }

// Glossy BSDF //

/*
Spectrum GlossyBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlossyBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0f;
  return reflect(wo, wi, reflectance);
}
*/

// Refraction BSDF //

    Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi,
                                      float* pdf) {
      refract(wo, wi, ior);
      *pdf = 1;

      return transmittance;
    }

// Glass BSDF //

    Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      // Compute Fresnel coefficient and either reflect or refract based on it.
      *pdf = 1.0f;

      if (!refract(wo, wi, ior)) {
        reflect(wo, wi);
        return reflectance * (1.0f / abs(wo.z));
      }

      double ni = 1.0, nt = ior;
      if (wo.z < 0) {
        //entering surface from vaccum
        swap(ni, nt);
      }
      double cosi = abs(wo.z), cost = abs(wi->z),
             cosi_x_ni = cosi * ni, cosi_x_nt = cosi * nt,
              cost_x_ni = cost * ni, cost_x_nt = cost * nt;

      double r_parallel = (cosi_x_nt - cost_x_ni) / (cosi_x_nt + cost_x_ni);
      double r_vertical = (cosi_x_ni - cost_x_nt) / (cosi_x_ni + cost_x_nt);
      double Fr = 0.5 * (r_parallel * r_parallel + r_vertical * r_vertical);
      double rand = (double)(std::rand()) / (double) RAND_MAX;

      if (rand <= Fr) {
        //reflectance
        reflect(wo, wi);
        return reflectance * (1.0f / abs(wo.z));
      }

      float weight = (float) (((nt * nt) / (ni * ni)) * (1.0f - Fr) * (1.0f / abs(wo.z)));
      return transmittance * weight;
    }

    void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {
      // V = 2 <U,N>N - U.
      *wi = Vector3D(-1.0f * wo.x, -1.0f * wo.y, wo.z).unit();
    }


    bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {
      // Use Snell's Law to refract wo surface and store result ray in wi.
      // Return false if refraction does not occur due to total internal reflection
      // and true otherwise. When dot(wo,n) is positive, then wo corresponds to a
      // ray entering the surface through enteringVacuum.

      Vector3D wo_copy = wo.unit();
      bool enteringVacuum = wo.z < 0;
      float ni = 1.0f, nt = ior, ratio;
      if (enteringVacuum) {
        //entering surface from enteringVacuum
        swap(ni, nt);
      }
      ratio = ni/ nt;
      float cosi = fabs((float) wo_copy.z), sini = sqrtf(1.0f - cosi * cosi);
      float sint = ratio * sini, cost = sqrtf(1.0f - sint * sint);
          if ((sini * sini) * ratio * ratio  >= 1.0f ) {
            return false;

          }
      Vector3D wii = Vector3D(wo_copy.x, wo_copy.y, 0);
      wii *= -1.0f * ni / nt;
      wii.z = cost * (enteringVacuum ? 1.0 : -1.0);
      wii.normalize();
      *wi = wii;

      return true;


    }

// Emission BSDF //

    Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
      return Spectrum();
    }

    Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
      *wi = sampler.get_sample(pdf);
      return Spectrum();
    }

}  // namespace PROJ6850