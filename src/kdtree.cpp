//
// Created by Yifei Li (liyifei@mit.edu) on 4/26/22.
//

#include "kdtree.h"
#include "static_scene/triangle.h"

#include <stack>
#include <cassert>
using namespace std;

namespace PROJ6850 {
    namespace StaticScene {


        KDTREEAccel::KDTREEAccel(const std::vector<Primitive *> &_primitives, size_t max_leaf_size) {

//           Construct a KD Tree from the given vector of primitives and maximum leaf
//           size configuration. The starter code build a kd-tree aggregate with a
//           single leaf node (which is also the root) that encloses all the
//           primitives.
          size_t totalNodeBuilt = 0;
          std::set<int> indices;
          BBox box;

          for (int i = 0; i < _primitives.size(); i++) {
            indices.insert(i);
            box.expand(_primitives[i]->get_bbox());
          }

          root = recursiveBuild( totalNodeBuilt, indices , _primitives, max_leaf_size, 0, box);
          primitives = _primitives;

        }

        KDTREEAccel::~KDTREEAccel() {
          // free any memories allocated as needed
        }

        double  findMedian(const std::vector<Primitive *> & originalPrimitives, std::set<int>& indices, int  splitAxis) {
          // First use naive sort algorithm for median finding
            std::vector<double> vals;
//          {
//            // spatial median
//            double min = originalPrimitives[0]->get_bbox().min[splitAxis];
//            double max = originalPrimitives[0]->get_bbox().max[splitAxis];
//            for (int idx : indices) {
//              BBox elemBBox = originalPrimitives[idx]->get_bbox();
//              min = std::min(min, elemBBox.min[splitAxis]);
//              max = std::max(max, elemBBox.max[splitAxis]);
//            }
//            return (min + max) / 2;
//          }


          for (int idx : indices) {
            BBox elemBBox = originalPrimitives[idx]->get_bbox();
            vals.emplace_back(elemBBox.centroid()[splitAxis]);
          }

          std::sort(vals.begin(), vals.end());
          if (vals.size() % 2 == 0) {
            // use middle value
            return (vals[vals.size() / 2] + vals[vals.size() / 2 - 1]) / 2.0;
          } else {
            return vals[vals.size() / 2];
          }


        }


        AccelNode *KDTREEAccel::recursiveBuild( size_t &totalNodesBuild, std::set<int>& indices,
                                               const std::vector<Primitive *> &originalPrimitives,
                                               size_t max_leaf_size, int level, BBox& box) {
          totalNodesBuild++;
          BBox boundBox; // boundBox for all primitives in this range
//          for (int idx : indices) {
//            BBox elemBBox = originalPrimitives[idx]->get_bbox();
//            boundBox.expand(elemBBox);
//          }

          boundBox = box;

          int N = indices.size();
          AccelNode *thisNode = new AccelNode(boundBox, indices);

          if ((N <= max_leaf_size) || (level >  8 + 1.3 * std::log(originalPrimitives.size())) ) {
            // build leaf node
            return thisNode;
          } else {
            BBox boundCentroidAll;
            int splitAxis;
            for (int idx : indices) {
              BBox box = originalPrimitives[idx]->get_bbox();
              boundCentroidAll.expand(box.centroid());
            }

            splitAxis = boundCentroidAll.longestDimension();

            double medianOnAxis = findMedian(originalPrimitives, indices, splitAxis);
            // split primitives on the splitaxis according to median, build left and right tree node
            AccelNode *leftNode, *rightNode;
            std::set<int> left, right;
            BBox leftBBox, rightBBox;
            leftBBox = boundBox;
            rightBBox = boundBox;
            leftBBox.max[splitAxis] = medianOnAxis;
            rightBBox.min[splitAxis] = medianOnAxis;

            for (int idx : indices) {
              BBox elemBBox = originalPrimitives[idx]->get_bbox();
              if (elemBBox.max[splitAxis] <= medianOnAxis) { // triangle should be put in the left box
                // add to left
                left.insert(idx);
              } else if (elemBBox.min[splitAxis] >= medianOnAxis)
              { // triangle intersects with right node
                // add to left
                right.insert(idx);
              }  else   {
                left.insert(idx);
                right.insert(idx);
              }
            }

            if ((left.size() == indices.size()) || (right.size() == indices.size())) // bad split, return tree node
              return thisNode;

            leftNode = recursiveBuild( totalNodesBuild, left, originalPrimitives,
                                      max_leaf_size, level + 1, leftBBox);
            rightNode = recursiveBuild(  totalNodesBuild, right, originalPrimitives,
                                       max_leaf_size, level + 1, rightBBox);

            thisNode->l = leftNode;
            thisNode->r = rightNode;
            return thisNode;
          }


        }


        void KDTREEAccel::traverse(const Ray &ray, AccelNode *currentNode, Intersection *isect, bool &hits) const {

          double t0 = 0, t1 = 0;
          if (currentNode == nullptr ||
              !currentNode->bb.intersect(ray, t0, t1)) { // no intersection with the bounding box
            return;
          }

          // if boox intersects, but
          if ((isect != nullptr) && (isect->t < t0)) {
            return;
          }

          if (currentNode->isLeaf()) {
            for (int idx : currentNode->indices) {
              if (((isect != nullptr) && primitives[idx]->intersect(ray, isect)) ||
                  ((isect == nullptr) && primitives[idx]->intersect(ray))) {
                hits = true;
              }
            }
          } else {
            double tminLeft = 0, tmaxLeft = 0, tminRight = 0, tmaxRight = 0;
            bool leftIntersect = (currentNode->l != nullptr) && (currentNode->l->bb.intersect(ray, tminLeft, tmaxLeft));
            bool rightIntersect =
                    (currentNode->r != nullptr) && (currentNode->r->bb.intersect(ray, tminRight, tmaxRight));

            // first traverse the node that has closer hit
            if (leftIntersect && rightIntersect) { //traversal optimization

              AccelNode* firstNode = currentNode->l;
              AccelNode* secondNode = currentNode->r;
              if (tminLeft >= tminRight) {
                firstNode = currentNode->r;
                secondNode = currentNode->l;
              }

              traverse(ray, firstNode, isect, hits);
              // If there is intersection with first node, and intersection point is within node, terminate search process
              bool skipSecondNode = false;
              if (hits) {
                Vector3D p = ray.o + ray.d * isect->t;
                if (firstNode->bb.isInside(p))
                  skipSecondNode = true;
              }
              if (!skipSecondNode)
                traverse(ray, secondNode, isect, hits);

            } else if (leftIntersect) {
              traverse(ray, currentNode->l, isect, hits);
            } else if (rightIntersect) {
              traverse(ray, currentNode->r, isect, hits);
            }
          }
        }

        bool KDTREEAccel::intersect(const Ray &ray, Intersection *isect) const {
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a kdtree aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate. When an intersection does happen.
          // You should store the non-aggregate primitive in the intersection data
          // and not the BVH aggregate itself.

          bool hit = false;
          traverse(ray, root, isect, hit);


          return hit;
        }

        bool KDTREEAccel::intersect(const Ray &ray) const {
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a kdtree aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate.
          bool hit = false;
          traverse(ray, root, nullptr, hit);
          return hit;
        }
    }
}