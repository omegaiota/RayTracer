#include <float.h>
#include <assert.h>
#include <unordered_map>
#include "meshEdit.h"
#include "mutablePriorityQueue.h"
#include "error_dialog.h"

namespace PROJ6850 {
    //Helper functions
    HalfedgeIter getPrev(HalfedgeIter self) {
      HalfedgeIter h = self, prev = self;
      do {
        prev = h;
        h = h->next();
      } while (h != self);
      return prev;
    }

    bool isTriangle(HalfedgeIter self) {
      FaceIter f = self->face();
      Size n = f->degree();

      return (n == 3);
    }

    bool isTwin(HalfedgeIter e0, HalfedgeIter e1) {
      return (e0->twin() == e1 && e1->twin() == e0);
    }

    bool faceIsLoop(FaceIter newF, Size n) {
      Size count = 0;
      HalfedgeIter curr = newF->halfedge();
//      printf("\nsize:%zu curr:%p\n", n, (void *) (curr->getHalfedge()));
      do {
        count++;
        assert(curr->face() == newF);
        curr = curr->next();
        assert(isTwin(curr, curr->twin()));
//        printf("count:%zu curr:%p\n", count, (void *) (curr->getHalfedge()));
        if (count != n && curr == newF->halfedge())
          return false;
      } while (count < n);
//      assert(curr->face() == newF);
      return (curr == newF->halfedge());
    }


    // Mesh Operations
    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
      // This method should split the given edge and return an iterator to the
      // newly inserted vertex. The halfedge of this vertex should point along
      // the edge that was split, rather than the new edges.
      if (e0->halfedge()->face()->degree() != 3 || e0->halfedge()->twin()->face()->degree() != 3) {
        showError("splitEdge() only supports triangle meshes");
        return VertexIter();
      }


      // name existing elements
      HalfedgeIter h0 = e0->halfedge(), h1 = h0->next(), h2 = h1->next(),
                   h3 = h0->twin(), h4 = h3->next(), h5 = h4->next(),
                   ou0 = h4->twin(), ou1 = h5->twin(), ou2 = h1->twin(), ou3 = h2->twin();
      VertexIter v0 = e0->halfedge()->vertex(), v1 = h5->vertex(),
                 v2 = h3->vertex(), v3 = h2->vertex();
      EdgeIter e4 = h2->edge(), e5 = h4->edge(), e6 = h5->edge(), e7 = h1->edge();
      FaceIter f0 = h0->face(), f1 = h3->face();

      //allocate new elements
      VertexIter v4 = newVertex();
      HalfedgeIter h6 = newHalfedge(), h7 = newHalfedge(), h8 = newHalfedge(), h9 = newHalfedge(),
          h10 = newHalfedge(), h11 = newHalfedge();
      EdgeIter e1 = newEdge(), e2 = newEdge(), e3 = newEdge();
      FaceIter f2 = newFace(), f3 = newFace();

      //reassign
      h0->setNeighbors(h1, h3, v0, e0, f0);
      h1->setNeighbors(h2, h11, v4, e3, f0);
      h2->setNeighbors(h0, ou3, v3, e4, f0);

      h3->setNeighbors(h4, h0, v4, e0, f1);
      h4->setNeighbors(h5, ou0, v0, e5, f1);
      h5->setNeighbors(h3, h7, v1, e1, f1);

      h6->setNeighbors(h7, h9, v2, e2, f2);
      h7->setNeighbors(h8, h5, v4, e1, f2);
      h8->setNeighbors(h6, ou1, v1, e6, f2);

      h9->setNeighbors(h10, h6, v4, e2, f3);
      h10->setNeighbors(h11, ou2, v2, e7, f3);
      h11->setNeighbors(h9, h1, v3, e3, f3);

      v0->halfedge() = h4;
      v1->halfedge() = h8;
      v2->halfedge() = h10;
      v3->halfedge() = h2;
      v4->halfedge() = h9;

      e0->halfedge() = h0;
      e1->halfedge() = h5;
      e2->halfedge() = h9;
      e3->halfedge() = h1;

      f0->halfedge() = h1;
      f1->halfedge() = h5;
      f2->halfedge() = h7;
      f3->halfedge() = h11;

      ou0->twin() = h4;
      ou1->twin() = h8;
      ou2->twin() = h10;
      ou3->twin() = h2;

//      assert(faceIsLoop(f0, 3));
//      assert(faceIsLoop(f1, 3));
//      assert(faceIsLoop(f2, 3));
//      assert(faceIsLoop(f3, 3));
//      assert(isTwin(h3, h0));
//      assert(isTwin(h5, h7));
//      assert(isTwin(h1, h11));
//      assert(isTwin(h6, h9));
//      assert(isTwin(ou0, h4));
//      assert(isTwin(ou1, h8));
//      assert(isTwin(ou2, h10));
//      assert(isTwin(ou3, h2));
      v4->position = (v0->position + v2->position) / 2;
      return v4;
    }

    VertexIter HalfedgeMesh::collapseEdge(EdgeIter e) {
      // This method should collapse the given edge and return an iterator to
      // the new vertex created by the collapse.

      //name existing elements
      HalfedgeIter h0 = e->halfedge();
      HalfedgeIter h1 = h0->next(),
              h2 = h1->next(),
              h3 = h0->twin(),
              h4 = h3->next(),
              h5 = h4->next(),
              h6 = h2->twin(),
              h7 = h6->next(),
              h8 = h7->next(),
              h9 = h4->twin(),
              h10 = h9->next(),
              h11 = h10->next();
      FaceIter f0 = h0->face(), f1 = h3->face(), f2 = h6->face(), f3 = h10->face();
      EdgeIter e0 = h0->edge(), e1 = h1->edge(), e2 = h5->edge(), e3 = h2->edge(), e4 = h4->edge();
      VertexIter v0 = h0->vertex(), v1 = h3->vertex(), v2 = h2->vertex(), v3 = h5->vertex();


      if ((v0->getVertex() == v1->getVertex())) {
        return v0;
      }
      if ((f0->degree() !=3) || (f1->degree() !=3)|| (f2->degree() !=3) || (f3->degree() !=3)) {
        showError("collapseEdge only supports triangle meshes");
        return h0->vertex();
      }

      Vector3D newPosition = (v0->position + v1->position) / 2;

      HalfedgeIter curr = v0->halfedge();
      do {
        assert(curr->vertex()->getVertex() == v0->getVertex());
        curr->vertex() = v1;
        curr = curr->twin()->next();
      } while (curr != v0->halfedge());


      h1->setNeighbors(h7, h1->twin(), h1->vertex(), h1->edge(), f2);
      h5->setNeighbors(h10, h5->twin(), h5->vertex(), h5->edge(), f3);
      h8->next() = h1;
      h11->next() = h5;

      v1->halfedge() = h1;
      v2->halfedge() = h7;
      v3->halfedge() = h5;

      f2->halfedge() = h7;
      f3->halfedge() = h11;
      if (f2->getFace() == f3->getFace()) {
        h5->next() = h1;
        h1->next() = h7;
        h7->next() = h5;
      }

//      printf("Collpase edge: delete halfedge: %p %p\n",(void *) h0->getHalfedge(),(void *) (h3->getHalfedge()));
      deleteHalfedge(h0);
      deleteHalfedge(h3);
      if (e3->getEdge() == e4->getEdge()) {
        deleteHalfedge(h4);
        deleteHalfedge(h2);
        deleteEdge(e3);

      } else {
        deleteHalfedge(h6);
        deleteHalfedge(h4);
        deleteHalfedge(h2);
        deleteHalfedge(h9);
        deleteEdge(e3);
        deleteEdge(e4);

      }


      deleteVertex(v0);

//      printf("Collpase edge: delete edge: %p \n",(void *) e0->getEdge());
      deleteEdge(e0);


      deleteFace(f0);
      deleteFace(f1);
//
//      std::cout << "collase edge newposition v1:" <<  v1->position << " v0:" << v0->position << " new:" << newPosition << "\n";
      v1->position = newPosition;
      return v1;
//      printf("h0: %p\n",(void *) (h0->getHalfedge())  );
//      printf("h1: %p\n",(void *) (h1->getHalfedge())  );
//      printf("h2: %p\n",(void *) (h2->getHalfedge())  );
//      printf("h3: %p\n",(void *) (h3->getHalfedge())  );
//      printf("h4: %p\n",(void *) (h4->getHalfedge())  );
//      printf("h5: %p\n",(void *) (h5->getHalfedge())  );
//      printf("h6: %p\n",(void *) (h6->getHalfedge())  );
//      printf("h7: %p\n",(void *) (h7->getHalfedge())  );
//      printf("h8: %p\n",(void *) (h8->getHalfedge())  );
//      printf("h9: %p\n",(void *) (h9->getHalfedge())  );
//      printf("h10: %p\n",(void *) (h10->getHalfedge())  );
//      printf("11: %p\n",(void *) (h11->getHalfedge())  );
//
//      printf("v0: %p\n  degree:%zu\n",(void *) (v0->getVertex()) , v0->degree() );
//      printf("v1: %p\n  degree:%zu\n",(void *) (v1->getVertex())  , v1->degree());
    }

    VertexIter HalfedgeMesh::collapseFace(FaceIter f) {
      // TODO: (meshEdit)
      // This method should collapse the given face and return an iterator to
      // the new vertex created by the collapse.
      showError("collapseFace() not implemented.");
      return VertexIter();
    }

    FaceIter HalfedgeMesh::eraseVertex(VertexIter v) {
      // TODO: (meshEdit)
      // This method should replace the given vertex and all its neighboring
      // edges and faces with a single face, returning the new face.

      return FaceIter();
    }

    FaceIter HalfedgeMesh::eraseEdge(EdgeIter e) {
      // TODO: (meshEdit)
      // This method should erase the given edge and return an iterator to the
      // merged face.

      showError("eraseVertex() not implemented.");
      return FaceIter();
    }


    void halfEdgeReassign(HalfedgeIter curr, HalfedgeIter next, HalfedgeIter twin, VertexIter vert, EdgeIter edge,
                          FaceIter f) {
      curr->next() = next;
      curr->twin() = twin;
      curr->vertex() = vert;
      curr->edge() = edge;
      curr->face() = f;
    }



    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
      // This method flips the given edge and return an iterator to the
      // flipped edge.
      if (e0->isBoundary()) {
        showError("Boundary edge cannot be flipped\n");
        return e0;
      }

      /* Name existing elements*/
      // HALFEDGES
      HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h4 = h3->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h7 = h2->twin();
      HalfedgeIter h8 = h4->twin();
      HalfedgeIter h9 = h5->twin();
      HalfedgeIter f0_last = getPrev(h0);
      HalfedgeIter f1_last = getPrev(h3);

      // VERTICES
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h3->vertex();
      VertexIter v2 = h5->vertex();
      VertexIter v3 = h2->vertex();

      // EDGES
      EdgeIter e1 = h5->edge();
      EdgeIter e2 = h4->edge();
      EdgeIter e3 = h2->edge();
      EdgeIter e4 = h1->edge();

      // FACES
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();


      /*---------Reassignment--------------*/
      // HALFEDGES
      if (!isTriangle(f0_last))
        f0_last->setNeighbors(h2, f0_last->twin(), f0_last->vertex(), f0_last->edge(), f0_last->face());
      if (!isTriangle(f1_last))
        f1_last->setNeighbors(h5, f1_last->twin(), f1_last->vertex(), f1_last->edge(), f1_last->face());
      //assert(h2->twin() == h8);

      h0->setNeighbors(h1, h3, v2, e0, f0);
      h1->setNeighbors(f0_last, h7, v3, e3, f0);
      h2->setNeighbors(h0, h8, v0, e2, f0);
      h3->setNeighbors(h4, h0, v3, e0, f1);
      h4->setNeighbors(f1_last, h9, v2, e1, f1);
      h5->setNeighbors(h3, h6, v1, e4, f1);

      //outside elements
      h6->setNeighbors(h6->next(), h5, v3, e4, h6->face());
      h7->setNeighbors(h7->next(), h1, h7->vertex(), e3, h7->face());
      h8->setNeighbors(h8->next(), h2, v2, e2, h8->face());
      h9->setNeighbors(h9->next(), h4, h9->vertex(), e1, h9->face());

// VERTICES
      v0->halfedge() = h2;
      v1->halfedge() = h5;
      v2->halfedge() = h4;
      v3->halfedge() = h3;

// EDGES
      e0->halfedge() = h0;
      e1->halfedge() = h4;
      e2->halfedge() = h2;
      e3->halfedge() = h1;
      e4->halfedge() = h5;

// FACES
      f0->halfedge() = h0;
      f1->halfedge() = h3;

      return e0;
    }

    void HalfedgeMesh::subdivideQuad(bool useCatmullClark) {
      if (useCatmullClark) {
        computeCatmullClarkPositions();
      } else {
        computeLinearSubdivisionPositions();
      }

      assignSubdivisionIndices();

      vector<vector<Index> > subDFaces;
      vector<Vector3D> subDVertices;
      subDFaces.clear();
      subDVertices.clear();
      buildSubdivisionFaceList(subDFaces);
      buildSubdivisionVertexList(subDVertices);

      rebuild(subDFaces, subDVertices);
    }

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * simple linear interpolation, e.g., the edge midpoints and face
 * centroids.
 */
    void HalfedgeMesh::computeLinearSubdivisionPositions() {
      // its original position, Vertex::position.
      for (Vertex &v : vertices) {
        v.newPosition = v.position;
      }

      // positions to Edge::newPosition.
      for (Edge &e : edges) {
        HalfedgeIter h = e.halfedge();
        e.newPosition = (h->vertex()->position + h->twin()->vertex()->position) / 2.0;
      }

      // of the original vertex positions to Face::newPosition.  Note
      // that in general, NOT all faces will be triangles!
      for (Face &f : faces) {
        HalfedgeIter curr = f.halfedge();
        Vector3D sumVertex = curr->vertex()->position;
        Size count = 1;

        while (curr->next() != f.halfedge()) {
          curr = curr->next();
          sumVertex += curr->vertex()->position;
          count++;
        }

        f.newPosition = sumVertex / count;
      }
    }

/**
 * Compute new vertex positions for a mesh that splits each polygon
 * into quads (by inserting a vertex at the face midpoint and each
 * of the edge midpoints).  The new vertex positions will be stored
 * in the members Vertex::newPosition, Edge::newPosition, and
 * Face::newPosition.  The values of the positions are based on
 * the Catmull-Clark rules for subdivision.
 */
    void HalfedgeMesh::computeCatmullClarkPositions() {
      // a lot like HalfedgeMesh::computeLinearSubdivisionPositions(),
      // except that the calculation of the positions themsevles is
      // slightly more involved, using the Catmull-Clark subdivision
      // rules. (These rules are outlined in the Developer Manual.)

      for (Face &f : faces) {
        HalfedgeIter curr = f.halfedge();
        Vector3D sumVertex = curr->vertex()->position;
        Size count = 1;

        while (curr->next() != f.halfedge()) {
          curr = curr->next();
          sumVertex += curr->vertex()->position;
          count++;
        }

//        assert (count == f.degree());
        f.newPosition = sumVertex / count;

      }

      for (Edge &e : edges) {
        HalfedgeIter h = e.halfedge();
        e.newPosition = (h->vertex()->position + h->twin()->vertex()->position
                         + h->face()->newPosition + h->twin()->face()->newPosition) / 4;
      }

      for (Vertex &v : vertices) {
        HalfedgeIter h = v.halfedge();
        Size n = v.degree(), count = 1;

        Vector3D Q = h->face()->newPosition;
        Vector3D R = h->edge()->newPosition;
        Vector3D S = v.position;

        while (h->twin()->next() != v.halfedge()) {
          h = h->twin()->next();
          Q += h->face()->newPosition;
          R += h->edge()->newPosition;

          count++;
        }

        Q = Q / n;
        R = R / n;
//        assert (count == n);
        v.newPosition = (Q + 2 * R + (n - 3) * S) / n;

      }

    }

/**
 * Assign a unique integer index to each vertex, edge, and face in
 * the mesh, starting at 0 and incrementing by 1 for each element.
 * These indices will be used as the vertex indices for a mesh
 * subdivided using Catmull-Clark (or linear) subdivision.
 */
    void HalfedgeMesh::assignSubdivisionIndices() {
      Index i = 0;
      for (Vertex &v : vertices)
        v.index = i++;

      for (Edge &e : edges)
        e.index = i++;

      for (Face &f : faces)
        f.index = i++;
    }

/**
 * Build a flat list containing all the vertex positions for a
 * Catmull-Clark (or linear) subdivison of this mesh.  The order of
 * vertex positions in this list must be identical to the order
 * of indices assigned to Vertex::newPosition, Edge::newPosition,
 * and Face::newPosition.
 */
    void HalfedgeMesh::buildSubdivisionVertexList(vector<Vector3D> &subDVertices) {
      subDVertices.reserve(vertices.size() + edges.size() + faces.size());
      // TODO Resize the vertex list so that it can hold all the vertices.

      // TODO Iterate over vertices, assigning Vertex::newPosition to the
      // appropriate location in the new vertex list.
      for  (Vertex v : vertices)
        subDVertices.push_back(v.newPosition);
      // TODO Iterate over edges, assigning Edge::newPosition to the appropriate
      // location in the new vertex list.
      for  (Edge e : edges)
        subDVertices.push_back(e.newPosition);

      // TODO Iterate over faces, assigning Face::newPosition to the appropriate
      // location in the new vertex list.
      for  (Face f : faces)
        subDVertices.push_back(f.newPosition);
    }

/**
 * Build a flat list containing all the quads in a Catmull-Clark
 * (or linear) subdivision of this mesh.  Each quad is specified
 * by a vector of four indices (i,j,k,l), which come from the
 * members Vertex::index, Edge::index, and Face::index.  Note that
 * the ordering of these indices is important because it determines
 * the orientation of the new quads; it is also important to avoid
 * "bowties."  For instance, (l,k,j,i) has the opposite orientation
 * of (i,j,k,l), and if (i,j,k,l) is a proper quad, then (i,k,j,l)
 * will look like a bowtie.
 */
    void HalfedgeMesh::buildSubdivisionFaceList(vector<vector<Index> > &subDFaces) {
      // TODO This routine is perhaps the most tricky step in the construction of
      // a subdivision mesh (second, perhaps, to computing the actual Catmull-Clark
      // vertex positions).  Basically what you want to do is iterate over faces,
      // then for each for each face, append N quads to the list (where N is the
      // degree of the face).  For this routine, it may be more convenient to simply
      // append quads to the end of the list (rather than allocating it ahead of
      // time), though YMMV.  You can of course iterate around a face by starting
      // with its first halfedge and following the "next" pointer until you get
      // back to the beginning.  The tricky part is making sure you grab the right
      // indices in the right order---remember that there are indices on vertices,
      // edges, AND faces of the original mesh.  All of these should get used.  Also
      // remember that you must have FOUR indices per face, since you are making a
      // QUAD mesh!

      // TODO iterate over faces
      for (Face originalF : faces) {
        // TODO loop around face
        HalfedgeIter h = originalF.halfedge();
        HalfedgeIter prevH = originalF.halfedge();
        do {
          prevH = prevH->next();
        } while (prevH->next() != originalF.halfedge());

        do {
          vector<Index> newQuad;
          newQuad.reserve(4);
          newQuad.push_back(h->vertex()->index);
          newQuad.push_back(h->edge()->index);
          newQuad.push_back(originalF.index);
          newQuad.push_back(prevH->edge()->index);

          subDFaces.push_back(newQuad);
//          printf("%p: %zu %zu %zu %zu\n",h,  newQuad[0], newQuad[1], newQuad[2], newQuad[3]);
//          assert((newQuad[0] != newQuad[1]) && (newQuad[2] != newQuad[3]) && (newQuad[1] != newQuad[2]));
          prevH = h;
          h = h->next();
        } while (h != originalF.halfedge());

      }
    }

    FaceIter HalfedgeMesh::bevelVertex(VertexIter v) {
      // TODO This method should replace the vertex v with a face, corresponding to
      // a bevel operation. It should return the new face.  NOTE: This method is
      // responsible for updating the *connectivity* of the mesh only---it does not
      // need to update the vertex positions.  These positions will be updated in
      // HalfedgeMesh::bevelVertexComputeNewPositions (which you also have to
      // implement!)

      showError("bevelVertex() not implemented.");
      return facesBegin();
    }

    FaceIter HalfedgeMesh::bevelEdge(EdgeIter e) {
      // TODO This method should replace the edge e with a face, corresponding to a
      // bevel operation. It should return the new face.  NOTE: This method is
      // responsible for updating the *connectivity* of the mesh only---it does not
      // need to update the vertex positions.  These positions will be updated in
      // HalfedgeMesh::bevelEdgeComputeNewPositions (which you also have to
      // implement!)

      showError("bevelEdge() not implemented.");
      return facesBegin();
    }

    FaceIter HalfedgeMesh::bevelFace(FaceIter f) {
      // (and ring of faces around it), corresponding to a bevel operation. It
      // should return the new face.  NOTE: This method is responsible for updating
      // the *connectivity* of the mesh only---it does not need to update the vertex
      // positions.  These positions will be updated in
      // HalfedgeMesh::bevelFaceComputeNewPositions (which you also have to
      // implement!)


      Size n = f->degree();
      HalfedgeIter curr = f->halfedge();


      FaceIter newF = newFace(); //new face


      FaceIter newRingF = newFace(), firstRFace = newRingF, prevRingF; //new face on the ring
      VertexIter newV = newVertex(), prevV;
      HalfedgeIter newH = newHalfedge(), newT = newHalfedge(), newRingH = newHalfedge(), newRingT = newHalfedge(),
              prevH, prevT, prevRingH, prevRingT;
      EdgeIter newE = newEdge(), prevE, newRingE = newEdge(), prevRingE;

      //Halfedge
      newH->setNeighbors(newH->next(), newT, newH->vertex(), newE, newF);
      newT->setNeighbors(newT->next(), newH, newV, newE, newRingF);
      newRingH->setNeighbors(newT, newRingT, curr->next()->vertex(), newRingE, newRingF);
      newRingT->setNeighbors(curr->next(), newRingH, newV, newRingE, newRingT->face());


      //Vertex
      newV->halfedge() = newH;

      //Edge
      newE->halfedge() = newH;
      newRingE->halfedge() = newRingH;

      //Face
      newF->halfedge() = newH;
      newRingF->halfedge() = newRingH;
//      printf("0 newH:%p\n", (void *) (newH->getHalfedge()));
//      printf("%d newT:%p\n", 0, (void *) (newT->getHalfedge()));
//      printf("%d newRingH:%p\n", 0, (void *) (newRingH->getHalfedge()));
//      printf("%d newRingT:%p\n", 0, (void *) (newRingT->getHalfedge()));
//      printf("%d curr:%p\n", 0, (void *) (curr->getHalfedge()));
      //updateOriginal connectivity
      for (Index i = 1; i < n; i++) {
        //store halfedge
        prevH = newH;
        prevRingH = newRingH;
        prevRingT = newRingT;

        //store vertex
        prevV = newV;

        //update outside
        HalfedgeIter prev = curr;
        curr = curr->next();
        prev->next() = prevRingH;
        prev->face() = prevRingH->face();
        prevV->position = curr->vertex()->position;

        //new assignments
        newH = newHalfedge();
//        printf("%zu newH:%p\n", i, (void *) (newH->getHalfedge()));
//        printf("%zu prevH:%p\n",i, (void *) (prevH->getHalfedge()));

        newT = newHalfedge();
        newRingH = newHalfedge();
        newRingT = newHalfedge();

        newV = newVertex();

        newE = newEdge();
        newRingE = newEdge();
        newRingF = newFace();
//        printf("%zu newT:%p\n", i, (void *) (newT->getHalfedge()));
//        printf("%zu newRingH:%p\n", i, (void *) (newRingH->getHalfedge()));
//        printf("%zu newRingT:%p\n", i, (void *) (newRingT->getHalfedge()));
//        printf("%zu curr:%p\n\n", i, (void *) (curr->getHalfedge()));

        //update leftover assignments for previous allocated elements
        prevH->next() = newH;
        prevH->face() = newF;
//        printf("%zu prevH->next:%p\n",i, (void *) (prevH->next()->getHalfedge()));

        prevRingT->face() = newRingF;

        //new assignments
        newH->setNeighbors(newH->next(), newT, prevV, newE, newF);
        newT->setNeighbors(prevRingT, newH, newV, newE, newRingF);
        newRingH->setNeighbors(newT, newRingT, curr->next()->vertex(), newRingE, newRingF);
        newRingT->setNeighbors(curr->next(), newRingH, newV, newRingE, newRingT->face());
        curr->face() = newRingF;

        //Vertex
        newV->halfedge() = newH;

        //Edge
        newE->halfedge() = newH;
        newRingE->halfedge() = newRingH;

        //Face
        newRingF->halfedge() = newRingH;
      }

      //assignment of last to first
      newH->next() = newF->halfedge();
      newH->face() = newF;
      newRingT->face() = firstRFace;
      curr->next() = newRingH;

      //assignment of first to last
      newF->halfedge()->vertex() = newV;
      newF->halfedge()->twin()->next() = newRingT;
      newV->position = f->halfedge()->vertex()->position;

      //check correctness
//      assert(faceIsLoop(newF, n));
      HalfedgeIter h = newF->halfedge();
//      do {
////        assert(faceIsLoop(h->twin()->face(), 4));
//        h = h->next();
//      } while (h != newF->halfedge());

      deleteFace(f);
      return newF;
    }

    void HalfedgeMesh::bevelFaceComputeNewPositions(
            vector<Vector3D> &originalVertexPositions,
            vector<HalfedgeIter> &newHalfedges, double normalShift,
            double tangentialInset) {

      Vector3D N(0., 0., 0.);
      Size size = originalVertexPositions.size();

      // calculate face normal
      for (int i = 0; i < size; i++) {
        Vector3D pi = originalVertexPositions[i];
        Vector3D pj = originalVertexPositions[(i + 1) % size];
        N += cross(pi, pj);
      }

      N = N.unit();
      for (int i = 0; i < newHalfedges.size(); i++) {
        Vector3D pi = originalVertexPositions[i]; //
        Vector3D pnext = originalVertexPositions[(i + 1) % size];
        Vector3D pprev = originalVertexPositions[(i + size - 1) % size];
        Vector3D tangential = (pprev - pi + pnext - pi).unit();
        tangential = tangential * tangentialInset;
        Vector3D delta = N * normalShift;
        Vector3D newP = newHalfedges[i]->vertex()->position + delta + tangential;
//        assert(!isnan(newP.x));
        newHalfedges[i]->vertex()->position = newP;

      }

    }

    void HalfedgeMesh::bevelVertexComputeNewPositions(
            Vector3D originalVertexPosition, vector<HalfedgeIter> &newHalfedges,
            double tangentialInset) {
      // TODO Compute new vertex positions for the vertices of the beveled vertex.
      //
      // These vertices can be accessed via newHalfedges[i]->vertex()->position for
      // i = 1, ..., hs.size()-1.
      //
      // The basic strategy here is to loop over the list of outgoing halfedges,
      // and use the preceding and next vertex position from the original mesh
      // (in the orig array) to compute an offset vertex position.

    }

    void HalfedgeMesh::bevelEdgeComputeNewPositions(
            vector<Vector3D> &originalVertexPositions,
            vector<HalfedgeIter> &newHalfedges, double tangentialInset) {
      // TODO Compute new vertex positions for the vertices of the beveled edge.
      //
      // These vertices can be accessed via newHalfedges[i]->vertex()->position for
      // i = 1, ..., newHalfedges.size()-1.
      //
      // The basic strategy here is to loop over the list of outgoing halfedges,
      // and use the preceding and next vertex position from the original mesh
      // (in the orig array) to compute an offset vertex position.
      //
      // Note that there is a 1-to-1 correspondence between halfedges in
      // newHalfedges and vertex positions
      // in orig.  So, you can write loops of the form
      //
      // for( int i = 0; i < newHalfedges.size(); i++ )
      // {
      //    Vector3D pi = originalVertexPositions[i]; // get the original vertex
      //    position correponding to vertex i
      // }
      //

    }

    void HalfedgeMesh::splitPolygons(vector<FaceIter> &fcs) {
      for (auto f : fcs) splitPolygon(f);
    }

    void HalfedgeMesh::splitPolygon(FaceIter f) {
      // Triangulate a polygonal face
//      printf("splitPoligon\n");
      if (f->halfedge()->next()->next()->next() == f->halfedge())
        return;
      Size n = f->degree();
      if (n == 3)
        return;

      HalfedgeIter curr = f->halfedge(), prevNewH, prev, newH, newT;
      VertexIter startV = curr->vertex();
      FaceIter newF;
      prevNewH = curr;
      prev = curr->next();
      curr = curr->next()->next(); // start at the third vertex
      EdgeIter newE;
      for (int i = 0; i < n - 3; i++) {
        newE = newEdge();
        newH = newHalfedge();
        newT = newHalfedge();
        VertexIter currV = curr->vertex();
        newF = newFace(); // new face for the previous halfedge + newT

        prevNewH->face() = newF;
        prev->face() = newF;
        prev->next() = newT;
        newH->setNeighbors(curr, newT, startV, newE, newH->face());
        newT->setNeighbors(prevNewH, newH, currV, newE, newF);

        newE->halfedge() = newH;
        newF->halfedge() = newT;
//        assert(newT->face() == newF);

        prevNewH = newH;
        prev = curr;
        curr = curr->next();
      }
        newF = newFace();
//      printf("newf:%p\n", addressof(newF));

        newF->halfedge() = curr;
        newH->face() = newF;
        prev->face() = newF;
        curr->face() = newF;

        curr->next() = prevNewH;

//      assert(faceIsLoop(newF, 3));
      HalfedgeIter h = newF->halfedge();
      int count = 0;

//      printf("splitPolygon deleted:%p\n", f->getFace());
      deleteFace(f);
    }

    EdgeRecord::EdgeRecord(EdgeIter &_edge) : edge(_edge) {
      // Compute the combined quadric from the edge endpoints.
      // -> Build the 3x3 linear system whose solution minimizes the quadric error
      //    associated with these two endpoints.
      // -> Use this system to solve for the optimal position, and store it in
      //    EdgeRecord::optimalPoint.
      // -> Also store the cost associated with collapsing this edg in
      //    EdgeRecord::Cost.
          this->edge =  _edge;
          VertexIter A = _edge->halfedge()->vertex(), B = _edge->halfedge()->twin()->vertex();
          Matrix4x4 sumQuad = A->quadric + B->quadric;
          double a[9] ={sumQuad.operator()(0,0),sumQuad.operator()(0,1),sumQuad.operator()(0,2),
                        sumQuad.operator()(1,0),sumQuad.operator()(1,1),sumQuad.operator()(1,2),
                        sumQuad.operator()(2,0),sumQuad.operator()(2,1),sumQuad.operator()(2,2)};
          Matrix3x3 matA = Matrix3x3(a);

      Vector3D b = Vector3D(sumQuad.operator()(0,3),sumQuad.operator()(1,3),sumQuad.operator()(2,3)) * (-1);

      Vector3D x =  matA.inv() * b;
          this->optimalPoint = x;
//          this->optimalPoint = (A->position + B->position) / 2.0;
//      std::cout << "A:" << matA << "\n";
//      std::cout << "b:" << b << "\n";
//      std::cout << "x:" << x << "\n";
//      std::cout  << "sumQuad:"<< sumQuad << "\n";
          double sc = dot(optimalPoint, sumQuad * optimalPoint);
          this->score = sc;
//          printf("score: %.2f\n", this->score);
    }

    void MeshResampler::upsample(HalfedgeMesh &mesh)
// This routine should increase the number of triangles in the mesh using Loop
// subdivision.
    {
      // TODO: (meshEdit)
      // Compute new positions for all the vertices in the input mesh, using
      // the Loop subdivision rule, and store them in Vertex::newPosition.
      // -> At this point, we also want to mark each vertex as being a vertex of the
      //    original mesh.
      // -> Next, compute the updated vertex positions associated with edges, and
      //    store it in Edge::newPosition.
      // -> Next, we're going to split every edge in the mesh, in any order.  For
      //    future reference, we're also going to store some information about which
      //    subdivided edges come from splitting an edge in the original mesh, and
      //    which edges are new, by setting the flat Edge::isNew. Note that in this
      //    loop, we only want to iterate over edges of the original mesh.
      //    Otherwise, we'll end up splitting edges that we just split (and the
      //    loop will never end!)
      // -> Now flip any new edge that connects an old and new vertex.
      // -> Finally, copy the new vertex positions into final Vertex::position.

      // Each vertex and edge of the original surface can be associated with a
      // vertex in the new (subdivided) surface.
      // Therefore, our strategy for computing the subdivided vertex locations is to
      // *first* compute the new positions
      // using the connectity of the original (coarse) mesh; navigating this mesh
      // will be much easier than navigating
      // the new subdivided (fine) mesh, which has more elements to traverse.  We
      // will then assign vertex positions in
      // the new mesh based on the values we computed for the original mesh.

      // Compute updated positions for all the vertices in the original mesh, using
      // the Loop subdivision rule.

      // Next, compute the updated vertex positions associated with edges.

      // Next, we're going to split every edge in the mesh, in any order.  For
      // future
      // reference, we're also going to store some information about which
      // subdivided
      // edges come from splitting an edge in the original mesh, and which edges are
      // new.
      // In this loop, we only want to iterate over edges of the original
      // mesh---otherwise,
      // we'll end up splitting edges that we just split (and the loop will never
      // end!)

      // Finally, flip any new edge that connects an old and new vertex.

      // Copy the updated vertex positions to the subdivided mesh.
      showError("upsample() not implemented.");
    }

    void MeshResampler::downsample(HalfedgeMesh &mesh) {
      // Compute initial quadrics for each face by simply writing the plane equation
      // for the face in homogeneous coordinates. These quadrics should be stored
      // in Face::quadric
      // -> Compute an initial quadric for each vertex as the sum of the quadrics
      //    associated with the incident faces, storing it in Vertex::quadric
      // -> Build a priority queue of edges according to their quadric error cost,
      //    i.e., by building an EdgeRecord for each edge and sticking it in the
      //    queue.
      // -> Until we reach the target edge budget, collapse the best edge. Remember
      //    to remove from the queue any edge that touches the collapsing edge
      //    BEFORE it gets collapsed, and add back into the queue any edge touching
      //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
      //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
      //    top of the queue.
      for (FaceIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
        Vector3D n = f->normal();
        double d = -dot(n, f->halfedge()->vertex()->position);
        Vector4D v =  Vector4D(n.x, n.y, n.z, d);
        f->quadric = outer(v, v);
      }

      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
        Matrix4x4 sumQuad;
        HalfedgeIter curr = v->halfedge();
        do {
          FaceIter f = curr->face();
          sumQuad += f->quadric;
          curr = curr->twin()->next();
        } while (curr != v->halfedge());
        v->quadric = sumQuad;
      }

      MutablePriorityQueue<EdgeRecord> edgeRecords;
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
        if ((e->halfedge()->vertex()->degree() > 2) && (e->halfedge()->twin()->vertex()->degree() > 2)) {
          EdgeRecord egR = EdgeRecord(e);
          e->getEdge()->record = egR;
          if (!isnan(egR.score)) {
            edgeRecords.insert(egR);
//            printf("Enqueue new edgeRecord for edge with edge: %p score: %.2f\n",(void *) e->getEdge(), egR.score);

          }
        }

      }

      Size count = 0;
      Size collapseNum = mesh.nFaces();
      while (count < collapseNum / 4) {
//        printf("count:%d\n", count);
        // get bestEdge to collapse
        if (edgeRecords.isEmpty())
          break;
        EdgeRecord bestEdge = edgeRecords.top();
        edgeRecords.pop();
        count++;


        // compute new quadric
        EdgeIter oldEdge = bestEdge.edge;
//        printf("\nRecord with edge %p to collpase\n",(void *) (oldEdge->getEdge()));
        VertexIter A = oldEdge->halfedge()->vertex(),
                B = oldEdge->halfedge()->twin()->vertex();
//        printf("vertice A:%p\n",(void *)(A->getVertex()));

//        printf("vertice B:%p\n",(void *)(B->getVertex()));
        bool sameVertex = ((void *)(A->getVertex())) == ((void *)(B->getVertex()));


        // remove any edge touching either of edge's endpoints from the queue
        for (int i = 0; i < 2; i++) {
          HalfedgeIter initial = (i == 0 ) ? A->halfedge() : B->halfedge();
          HalfedgeIter curr = initial;
          do {
            EdgeIter e = curr->edge();
            Edge* edge = e->getEdge();
              edgeRecords.remove(edge->record);
            curr = curr->twin()->next();
          } while (curr != initial);
        }

        if (sameVertex || (A->degree() < 3) || (B->degree() < 3)) {
          continue;

        }
        Matrix4x4 sumQuad = A->quadric + B->quadric;

        // collapse edge
        void* add = addressof(oldEdge->halfedge());
//        printf("\nCalling collapseEdge on edge %p\n",(void *)(oldEdge->getEdge()));
        VertexIter newV = mesh.collapseEdge(oldEdge);
        if (!(isnan(bestEdge.optimalPoint.x) || isnan(bestEdge.optimalPoint.y) || isnan(bestEdge.optimalPoint.z)))
          newV->position = bestEdge.optimalPoint;
        Matrix4x4 oldQuad = newV->quadric;

        // update quadric

        // insert new record touching new vertex
        HalfedgeIter curr = newV->halfedge();
        if (newV->degree() > 2)  {
          newV->quadric = sumQuad;
          do {
            EdgeIter e = curr->edge();
            Edge* edge = e->getEdge();
            EdgeRecord newRecord = EdgeRecord(e);
            if (!isnan(newRecord.score))
              edgeRecords.insert(newRecord);
            e->record = newRecord;

            curr = curr->twin()->next();
          } while (curr != newV->halfedge());

        }


      }

    }

    void MeshResampler::resample(HalfedgeMesh &mesh) {
      // TODO: (meshEdit)
      // Compute the mean edge length.
      // Repeat the four main steps for 5 or 6 iterations
      // -> Split edges much longer than the target length (being careful about
      //    how the loop is written!)
      // -> Collapse edges much shorter than the target length.  Here we need to
      //    be EXTRA careful about advancing the loop, because many edges may have
      //    been destroyed by a collapse (which ones?)
      // -> Now flip each edge if it improves vertex degree
      // -> Finally, apply some tangential smoothing to the vertex positions
      showError("resample() not implemented.");
    }

}  // namespace PROJ6850
