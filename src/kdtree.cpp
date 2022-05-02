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

        bool comparePrimitive(Primitive* prim1, Primitive* prim2, size_t dimension) {
          return prim1->get_bbox().centroid()[dimension] <= prim2->get_bbox().centroid()[dimension];
        }
        bool compareX(Primitive* prim1, Primitive* prim2) {
          return comparePrimitive(prim1, prim2, 0);
        }
        bool compareY(Primitive* prim1, Primitive* prim2) {
          return comparePrimitive(prim1, prim2, 1);
        }
        bool compareZ(Primitive* prim1, Primitive* prim2) {
          return comparePrimitive(prim1, prim2, 2);
        }
        KDTREEAccel::KDTREEAccel(const std::vector<Primitive *> &_primitives, size_t max_leaf_size) {

//           Construct a KD Tree from the given vector of primitives and maximum leaf
//           size configuration. The starter code build a kd-tree aggregate with a
//           single leaf node (which is also the root) that encloses all the
//           primitives.
          size_t totalNodeBuilt = 0;
          std::vector<Primitive *> orderedPrimitives;
          orderedPrimitives.reserve(_primitives.size());
          std::vector<std::vector<Primitive *>> sortedPrimitives;
          for (size_t i = 0; i < 3; i++) {
            std::vector<Primitive *> temp;
            temp.reserve(_primitives.size()); 
            for (size_t j = 0; j < _primitives.size(); j++) {
              temp.push_back(_primitives[j]);
            }
            sortedPrimitives.push_back(temp);
          }
          sort(sortedPrimitives[0].begin(), sortedPrimitives[0].end(), compareX);
          sort(sortedPrimitives[1].begin(), sortedPrimitives[1].end(), compareY);
          sort(sortedPrimitives[2].begin(), sortedPrimitives[2].end(), compareZ);

          root = KDTREEAccel::recursiveBuild(0, _primitives.size(), totalNodeBuilt, sortedPrimitives, orderedPrimitives, max_leaf_size, 0);
          assert(root->range == _primitives.size());
          assert(orderedPrimitives.size() == root->range);
          primitives = orderedPrimitives;
        }

        KDTREEAccel::~KDTREEAccel() {
          // free any memories allocated as needed
        }

        AccelNode *KDTREEAccel::recursiveBuild(size_t start, size_t end, size_t &totalNodesBuild,
                                               std::vector<std::vector<Primitive *>> &sortedPrimitives,
                                               std::vector<Primitive *> &orderedPrimitives,
                                               size_t max_leaf_size, size_t splitDimension) {
          cout << 1 << endl;
          assert(splitDimension >= 0 && splitDimension <= 2);
          totalNodesBuild++;
          size_t rangeCount = end - start;
          BBox boundBox; // boundBox for all primitives in this rangeCount
          for (size_t i = 0; i < rangeCount; i++) {
//            cout << "Here" << endl;
//            cout << sortedPrimitives[0][i]->get_bbox().centroid()[0] << endl;
//            cout << sortedPrimitives[0][i]->get_bbox().centroid()[1] << endl;
//            cout << sortedPrimitives[0][i]->get_bbox().centroid()[2] << endl;
//            cout << "Here" << endl;
            boundBox.expand(sortedPrimitives[0][i]->get_bbox());
          }
          cout << 2 << endl;
          AccelNode *thisNode = new AccelNode(boundBox, orderedPrimitives.size(), rangeCount); // TODO: still confused why this isn't sortedPrimitives[0].size()
          cout << 3 << endl;

          if (rangeCount <= max_leaf_size) {
            // Leafnode
            for (size_t i = 0; i < rangeCount; i++) {
              orderedPrimitives.push_back(sortedPrimitives[splitDimension][i]);
            }
            return thisNode;
          }
          cout << 4 << endl;

          // Recursive Case
          size_t middle_index = (end + start) / 2; // TODO: this is int division?
          Vector3D split_coor = sortedPrimitives[splitDimension][middle_index]->get_bbox().centroid();
          cout << 5 << endl;

          std::vector<Primitive *> first_half;
          std::vector<Primitive *> second_half;
          std::vector<Primitive *> left_ordered1;
          std::vector<Primitive *> right_ordered1;
          std::vector<Primitive *> left_ordered2;
          std::vector<Primitive *> right_ordered2;
          first_half.reserve(sortedPrimitives[0].size());
          second_half.reserve(sortedPrimitives[0].size());
          left_ordered1.reserve(sortedPrimitives[0].size());
          right_ordered1.reserve(sortedPrimitives[0].size());
          left_ordered2.reserve(sortedPrimitives[0].size());
          right_ordered2.reserve(sortedPrimitives[0].size());
          std::vector<Primitive *> ordered1 = sortedPrimitives[(splitDimension + 1) % 3];
          std::vector<Primitive *> ordered2 = sortedPrimitives[(splitDimension + 2) % 3];
          cout << 6 << endl;
          
          // This isn't necessarily balanced because multiple coordinates could have same x, y, or z
          for (int i = 0; i < sortedPrimitives[0].size(); i++) {
            if (ordered1[i]->get_bbox().centroid()[splitDimension] <= split_coor[splitDimension]) {
              left_ordered1.push_back(ordered1[i]);
            } else {
              right_ordered1.push_back(ordered1[i]);
            }
            if (ordered2[i]->get_bbox().centroid()[splitDimension] <= split_coor[splitDimension]) {
              left_ordered2.push_back(ordered2[i]);
            } else {
              right_ordered2.push_back(ordered2[i]);
            }
            if (sortedPrimitives[splitDimension][i]->get_bbox().centroid()[splitDimension] <= split_coor[splitDimension]) {
              first_half.push_back(sortedPrimitives[splitDimension][i]);
            } else {
              second_half.push_back(sortedPrimitives[splitDimension][i]);
            }
          }
          cout << 7 << endl;

          std::vector<std::vector<Primitive *>> new_sortedPrimitivesLeft;
          std::vector<std::vector<Primitive *>> new_sortedPrimitivesRight;
          for (size_t i = 0; i < 3; i++) {
              new_sortedPrimitivesLeft.push_back({});
              new_sortedPrimitivesRight.push_back({});
              new_sortedPrimitivesLeft[i].reserve(first_half.size()); // Could add count to for loop above and only reserve necessary space; space blows up rn
              new_sortedPrimitivesRight[i].reserve(second_half.size());
          }
          cout << 8 << endl;
          // These two for loops can be combined some way
          for (size_t i = 0; i < first_half.size(); i++) {
            new_sortedPrimitivesLeft[splitDimension].push_back(first_half[i]);
            new_sortedPrimitivesLeft[(splitDimension + 1) % 3].push_back(left_ordered1[i]);
            new_sortedPrimitivesLeft[(splitDimension + 2) % 3].push_back(right_ordered1[i]);
          }
          cout << 9 << endl;
          for (size_t i = 0; i < second_half.size(); i++) {
            new_sortedPrimitivesRight[splitDimension].push_back(second_half[i]);
            new_sortedPrimitivesRight[(splitDimension + 1) % 3].push_back(left_ordered2[i]);
            new_sortedPrimitivesRight[(splitDimension + 2) % 3].push_back(right_ordered2[i]);
          }
          cout << 10 << endl;
          assert(start + new_sortedPrimitivesLeft[0].size() + new_sortedPrimitivesRight[0].size() == end);
          thisNode->l = recursiveBuild(start, start + new_sortedPrimitivesLeft[0].size(), totalNodesBuild, new_sortedPrimitivesLeft, orderedPrimitives, max_leaf_size, (splitDimension + 1) % 3);
          thisNode->r = recursiveBuild(start + new_sortedPrimitivesLeft[0].size(), end, totalNodesBuild, new_sortedPrimitivesRight, orderedPrimitives, max_leaf_size, (splitDimension + 1) % 3);
          cout << "Finished" << endl;
          return thisNode;
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
            for (size_t i = currentNode->start; i < currentNode->start + currentNode->range; i++) {
              if (((isect != nullptr) && primitives[i]->intersect(ray, isect)) ||
                  ((isect == nullptr) && primitives[i]->intersect(ray))) {
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
              if (tminLeft < tminRight) {
                traverse(ray, currentNode->l, isect, hits);
                traverse(ray, currentNode->r, isect, hits);
              } else {

                traverse(ray, currentNode->r, isect, hits);
                traverse(ray, currentNode->l, isect, hits);
              }
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