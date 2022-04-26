#ifndef PROJ6850_MESHEDIT_H
#define PROJ6850_MESHEDIT_H

#include "halfEdgeMesh.h"

using namespace std;

namespace PROJ6850 {

class MeshResampler {
 public:
  MeshResampler(){};
  ~MeshResampler() {}

  void upsample(HalfedgeMesh& mesh);
  void downsample(HalfedgeMesh& mesh);
  void resample(HalfedgeMesh& mesh);
};

}  // namespace PROJ6850

#endif  // PROJ6850_MESHEDIT_H
