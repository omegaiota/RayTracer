#include "triangle.h"

#include "PROJ6850/PROJ6850.h"
#include "GL/glew.h"

namespace PROJ6850 {
    namespace StaticScene {

        Triangle::Triangle(const Mesh* mesh, vector<size_t>& v) : mesh(mesh), v(v) {}
        Triangle::Triangle(const Mesh* mesh, size_t v1, size_t v2, size_t v3)
                : mesh(mesh), v1(v1), v2(v2), v3(v3) {}

        BBox Triangle::get_bbox() const {
          // TODO (PathTracer):
          Vector3D p1 = mesh->positions[v1], p2 = mesh->positions[v2], p3 = mesh->positions[v3];
          Vector3D pMax = Vector3D( max(p1.x, p2.x, p3.x),  max(p1.y, p2.y, p3.y),  max(p1.z, p2.z, p3.z));
          Vector3D pMin = Vector3D( min(p1.x, p2.x, p3.x),  min(p1.y, p2.y, p3.y),  min(p1.z, p2.z, p3.z));
          return BBox(pMin, pMax);
        }

        double Triangle::max(double a, double b, double c) const {
          if (a > b && a > c)
            return a;
          if (b > c)
            return b;
          return c;
        }

        double Triangle::min(double a, double b, double c) const {
          if (a < b && a < c)
            return a;
          if (b < c)
            return b;
          return c;
        }

        bool Triangle::getIntersectInfo(const Ray &r, double &u, double &v, double &t) const{
          Vector3D p0 = mesh->positions[v1], p1 = mesh->positions[v2], p2 = mesh->positions[v3];
          Vector3D e1 = p1 - p0, e2 = p2 - p0,
                  s = r.o - p0,
                  s_x_e2 = cross(s, e2),
                  e1_x_d = cross(e1, r.d);
          double u_x_area = -1.0 * dot(s_x_e2, r.d),
          v_x_area = dot(e1_x_d , s),
          t_x_ara = -1.0 * dot(s_x_e2, e1);
          double area = dot(e1_x_d, e2);

          // edge case: ray is parallel to the plane of the triangle
          if (area == 0) {
            return false;
          }
            u = u_x_area / area;
            v = v_x_area / area;
            t = t_x_ara / area;
            bool isValidTime = t <= r.max_t && t>= r.min_t;
            return ((isValidTime) && (u >= 0.0) && (v >= 0.0) && (u <= 1.0) && (v <= 1.0) && (u + v <= 1.0 ));
        }

        bool Triangle::intersect(const Ray& r) const {
          double u = 0.0,v = 0.0,t = 0.0;
          return (getIntersectInfo(r, u, v, t));
        }

        bool Triangle::intersect(const Ray& r, Intersection* isect) const {
          double u = 0.0,v = 0.0,t = 0.0; // u is barycentric coordinate of
          if (getIntersectInfo(r, u, v, t)) {
            if (isect->t > t) {
              r.max_t = t;
              Vector3D interpolatedNormal =  u * mesh->normals[v2] + v * mesh->normals[v3] + (1.0-u-v) * mesh->normals[v1];
              interpolatedNormal.normalize();
              if (dot(interpolatedNormal, r.d) > 0) {
                // intersection occurs at the back
                interpolatedNormal *= -1.0;
              }
              isect->t = t;
              isect->n = interpolatedNormal;
              isect->primitive = this;
              isect->bsdf = get_bsdf();
              return true;
            }
            return false;
          }
          return false;
        }

        void Triangle::draw(const Color& c) const {
          glColor4f(c.r, c.g, c.b, c.a);
          glBegin(GL_TRIANGLES);
          glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
                     mesh->positions[v1].z);
          glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
                     mesh->positions[v2].z);
          glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
                     mesh->positions[v3].z);
          glEnd();
        }

        void Triangle::drawOutline(const Color& c) const {
          glColor4f(c.r, c.g, c.b, c.a);
          glBegin(GL_LINE_LOOP);
          glVertex3d(mesh->positions[v1].x, mesh->positions[v1].y,
                     mesh->positions[v1].z);
          glVertex3d(mesh->positions[v2].x, mesh->positions[v2].y,
                     mesh->positions[v2].z);
          glVertex3d(mesh->positions[v3].x, mesh->positions[v3].y,
                     mesh->positions[v3].z);
          glEnd();
        }

    }  // namespace StaticScene
}  // namespace PROJ6850