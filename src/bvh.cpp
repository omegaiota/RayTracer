#include "bvh.h"

#include "static_scene/triangle.h"

#include <stack>
#include <cassert>

using namespace std;

namespace PROJ6850 {
    namespace StaticScene {


        BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives, size_t max_leaf_size) {

//           Construct a BVH from the given vector of primitives and maximum leaf
//           size configuration. The starter code build a BVH aggregate with a
//           single leaf node (which is also the root) that encloses all the
//           primitives.
          size_t totalNodeBuilt = 0;
          std::vector<Primitive *> orderedPrimitives;
          orderedPrimitives.reserve(_primitives.size());

          root = recursiveBuild(0, _primitives.size(), totalNodeBuilt, orderedPrimitives, _primitives, max_leaf_size);
          assert(root->range == _primitives.size());
          assert(orderedPrimitives.size() == root->range);
          primitives = orderedPrimitives;


        }

        AccelNode *BVHAccel::recursiveBuild(size_t start, size_t end, size_t &totalNodesBuild,
                                            std::vector<Primitive *> &orderedPrimitives,
                                            const std::vector<Primitive *> &originalPrimitives,
                                            size_t max_leaf_size) {
           totalNodesBuild++;
          size_t range = end - start;

          BBox boundBox; // boundBox for all primitives in this range
          for (size_t i = start; i < end; i++) {
            boundBox.expand(originalPrimitives[i]->get_bbox());
          }
          AccelNode *thisNode = new AccelNode(boundBox, orderedPrimitives.size(), range);

           if (range <= max_leaf_size) {
            // Leafnode
            for (size_t i = start; i < end; i++) {
              orderedPrimitives.push_back(originalPrimitives[i]);
            }
            return thisNode;
          }

          // interior node, needs recursive call
          // compute bounding box for all primitives' centroid, choose split dimension
          // according to the axis that has maximum bounding box extent
          BBox boundCentroidAll;
          int splitDimension;
          for (size_t i = start; i < end; i++) {
            boundCentroidAll.expand(originalPrimitives[i]->get_bbox().centroid());
          }
          splitDimension = boundCentroidAll.longestDimension();

          assert(splitDimension >= 0 && splitDimension <= 2);
          // partition into two sets using SAH
          size_t mid = (start + end) / 2;

          // allocate SAH buckets
          const int BUCKET_NUM = 12;
          struct BucketInfo {
              BucketInfo() { count = 0; }
              int count;
              BBox bound;
              std::vector<Primitive *> prims;
          };
          BucketInfo buckets[BUCKET_NUM];
          for (int i = 0; i < BUCKET_NUM; i++) {
            buckets[i].prims.clear();
            buckets[i].prims.reserve(range);
          }
          // put primitives into corresponding bucket
          double maxDimensionRange = boundCentroidAll.extent[splitDimension];
          for (size_t i = start; i < end; i++) {
            int bucketIndex =
                    (int) (BUCKET_NUM * ((originalPrimitives[i]->get_bbox().centroid()[splitDimension] -
                                          boundCentroidAll.min[splitDimension])
                                         / maxDimensionRange));
            if (bucketIndex >= BUCKET_NUM)
              bucketIndex = BUCKET_NUM - 1;
            buckets[bucketIndex].bound.expand(originalPrimitives[i]->get_bbox());
            buckets[bucketIndex].count++;
            buckets[bucketIndex].prims.push_back(originalPrimitives[i]);
          }

          // find the best bucket for partition
          double cost[BUCKET_NUM - 1];
          std::fill_n(cost, BUCKET_NUM - 1, 0.0f);
          for (int i = 0; i < BUCKET_NUM - 1; i++) {
            BBox leftBox, rightBox;
            int pLEFT = 0, pRIGHT = 0;
            for (int j = 0; j <= i; j++) {
              leftBox.expand(buckets[j].bound);
              pLEFT++;
            }

            for (int j = i + 1; j < BUCKET_NUM; j++) {
              rightBox.expand(buckets[j].bound);
              pRIGHT++;
            }
            cost[i] = 0.125 + (pLEFT * leftBox.surface_area() + pRIGHT * rightBox.surface_area()) / boundBox.surface_area();
          }

          size_t minBucketSplit = 0;
          for (size_t i = 0; i < BUCKET_NUM - 1; i++) {
            if (cost[i] < cost[minBucketSplit]) {
              minBucketSplit = i;
            }
          }

          // split primitives
          AccelNode *leftNode, *rightNode;
          if (cost[minBucketSplit] < range) {
            std::vector<Primitive *> left, right;
            left.clear();
            right.clear();


            size_t countLeft = 0;
            for (int i = 0; i <= minBucketSplit; i++) {
              countLeft += buckets[i].count;
            }
            left.reserve(countLeft);
            right.reserve(range - countLeft);
            for (int i = 0; i <= minBucketSplit; i++) {
              for (int j = 0; j < buckets[i].count; j++) {
                left.push_back(buckets[i].prims[j]);
              }
            }
            for (size_t i = minBucketSplit + 1; i < BUCKET_NUM; i++) {
              for (int j = 0; j < buckets[i].count; j++) {
                right.push_back(buckets[i].prims[j]);
              }
            }

            leftNode = recursiveBuild(0, countLeft, totalNodesBuild, orderedPrimitives, left,
                                      max_leaf_size);
            rightNode = recursiveBuild(0, range - countLeft, totalNodesBuild, orderedPrimitives, right,
                                       max_leaf_size);
          } else {
            leftNode = recursiveBuild(start, mid, totalNodesBuild, orderedPrimitives, originalPrimitives,
                                      max_leaf_size);
            rightNode = recursiveBuild(mid, end, totalNodesBuild, orderedPrimitives, originalPrimitives,
                                       max_leaf_size);
          }
          thisNode->l = leftNode;
          thisNode->r = rightNode;
          return thisNode;


        }



        void BVHAccel::traverse(const Ray &ray, AccelNode *currentNode, Intersection *isect, bool &hits) const {

          double t0 = 0, t1 = 0;
          if (currentNode == nullptr || !currentNode->bb.intersect(ray, t0, t1)) {

            return;
          }

          if (isect != nullptr && isect->t < t0) {
//            printf("early out 2\n");
            return;
          }

          if (currentNode->isLeaf()) {
//            printf("this is a leaf node");
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

        bool BVHAccel::intersect(const Ray &ray, Intersection *isect) const {
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a BVH aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate. When an intersection does happen.
          // You should store the non-aggregate primitive in the intersection data
          // and not the BVH aggregate itself.

          bool hit = false;
          traverse(ray, root, isect, hit);


          return hit;
        }

        bool BVHAccel::intersect(const Ray &ray) const {
          // Implement ray - bvh aggregate intersection test. A ray intersects
          // with a BVH aggregate if and only if it intersects a primitive in
          // the BVH that is not an aggregate.
          bool hit = false;
          traverse(ray, root, nullptr, hit);
          return hit;
        }


        void recursiveDelete(AccelNode* node) {
          if (node->isLeaf()) {
            delete node;
          } else {
            recursiveDelete(node->l);
            recursiveDelete(node->r);
          }
        }

        BVHAccel::~BVHAccel() {
          // Implement a proper destructor for your BVH accelerator aggregate
          recursiveDelete(root);
        }


    }  // namespace StaticScene
}  // namespace PROJ6850