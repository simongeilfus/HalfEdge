/*
 Copyright (c) 2017, Simon Geilfus
 All rights reserved.
 
 This code is designed for use with the Cinder C++ library, http://libcinder.org
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that
 the following conditions are met:
    * Redistributions of source code must retain the above copyright notice, this list of conditions and
	the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
	the following disclaimer in the documentation and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <vector>
#include <memory>

#include "cinder/Plane.h"
#include "cinder/Vector.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Exception.h"

// cinder forward declarations
namespace cinder {
class TriMesh; class Ray; class AxisAlignedBox; 
namespace gl { typedef std::shared_ptr<class VboMesh> VboMeshRef; }
}
namespace ci = cinder;

namespace halfedge {

// Forward declarations
class HalfEdge; class Edge; class Vertex; class Face; 
class MeshSource; class MeshOperation;
enum class MeshFlag : uint8_t;
// Mesh Type alias
using MeshRef = std::shared_ptr<class Mesh>;

//! Mesh class
class Mesh {
public:
	//! Creates and returns an empty ref-counted Mesh object
	static MeshRef create();
	//! Creates and returns a ref-counted Mesh object from another Mesh
	static MeshRef create( const Mesh &mesh );
	//! Creates and returns a ref-counted Mesh object from a MeshSource primitive object
	static MeshRef create( const MeshSource &source );
	//! Creates and returns a ref-counted Mesh object from a list of vertices and a list of indices
	static MeshRef create( const std::vector<ci::vec3> &vertices, const std::vector<uint32_t> &indices );
	//! Creates and returns a ref-counted Mesh object from lists of faces and a list of vertices
	static MeshRef create( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces );
	//! Creates and returns a ref-counted Mesh object from a ci::TriMesh
	static MeshRef create( const ci::TriMesh &trimesh );
	
	//! Constructs an empty Mesh object
	Mesh();
	//! Constructs a Mesh object from another Mesh
	Mesh( const Mesh &mesh );
	//! Constructs a Mesh Mesh object from a MeshSource primitive object
	Mesh( const MeshSource &source );
	//! Constructs a Mesh object from a list of vertices and a list of indices
	Mesh( const std::vector<ci::vec3> &vertices, const std::vector<uint32_t> &indices );
	//! Constructs a Mesh object from lists of faces and a list of vertices
	Mesh( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces );
	//! Constructs a Mesh object from another Mesh
	Mesh( const ci::TriMesh &trimesh );
	
	//! Deep copy a Mesh from another Mesh
    const Mesh& operator=( const Mesh &mesh );

	//! Splits a face by connecting two vertices together and returns the new Edge.
	Edge*	splitFace( Face* face, Vertex* a, Vertex* b );
	//! Splits a face by connecting two vertices created at the intersection with a plane and returns the new Edge.
	Edge*	splitFace( Face* face, const ci::Planef &plane );
	//! Splits a face by connecting two vertices created at the intersection with a plane and returns the new Edge.
	Edge*	splitFace( Face* face, const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal );
	//! Splits a face by adding a vertex in its center and connecting it to all the other vertices.
	Vertex*	splitFace( Face* face );
	//! Splits a face by connecting two vertices created at the intersection with a plane and returns the new Edge.
	void	splitFaces( const ci::Planef &plane );
	//! Splits a face by connecting two vertices created at the intersection with a plane and returns the new Edge.
	void	splitFaces( const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal );

	//! Splits an edge in two by adding a vertex in between and returns the new Vertex.
	Vertex* splitEdge( Edge* edge, float t = 0.5f );
	//! Splits an edge in two by adding a vertex in between and returns the new Vertex.
	Vertex* splitEdge( Edge* edge, const ci::vec3 &splitPosition );
	
	//! Splits a vertex into two vertices by adding an edge between them, the specified list of edge follow the new vertex.
	Vertex* splitVertex( Vertex* vertex, Edge* firstEdge, Edge* lastEdge = nullptr );
	//! Splits a vertex into two vertices by adding an edge between them, the specified list of edge follow the new vertex.
	Vertex* splitVertex( Vertex* vertex, HalfEdge* firstHalfEdge, HalfEdge* lastHalfEdge = nullptr );
	//! Splits a vertex into two vertices by adding an edge between them. The resulting vertex is a floating/dangling vertex (connected only to the input vertex).
	Vertex* splitVertex( Vertex* vertex, Face* face );

	//! Collapses an edge by joining its two vertices together and returns the remaining Vertex.
	Vertex*	collapseEdge( Edge* edge, float t = 0.5f );
	//! Collapses an edge by joining two vertices together and returns the remaining Vertex.
	Vertex* collapseEdge( Vertex* vertex0, Vertex* vertex1, float t = 0.5f );
	// toward halfedge and not toward center of edge
	Vertex*	collapseHalfEdge( HalfEdge* halfEdge, float t = 0.0f );
	
	class Extrusion;

	//! Extrudes a face along the direction vector
	void extrude( Face* face, const Extrusion &extrusion );
	//! Extrudes a face along the direction vector
	void extrude( Face* face, float offset );
	//! Inner-Extrudes a face toward its centroid
	void extrudeInner( Face* face, float offset );
	//! Extrudes a group of faces along the direction vector
	void extrude( const std::vector<Face*> &faces, const Extrusion &extrusion );
	//! Extrudes a group of faces along the direction vector
	void extrude( const std::vector<Face*> &faces, float offset );
	//! Inner-Extrudes a group of faces toward their respective centroids
	void extrudeInner( const std::vector<Face*> &faces, float offset );

	//! Truncates a Vertex by creating a Face by all its connected HalfEdges (Similar to bevel but for vertices instead of edges).
	Face* truncate( Vertex* vertex, float radius );
	
	class Bevel;

	//! Bevels an Edge
	Face* bevel( Edge* edge, const Bevel &bevel );
	//! Bevels an Edge
	Face* bevel( Edge* edge, float radius );
	
	//! Applies a transformation to the geometry, leaving the topology intact
	void transform( const ci::mat4 &mat );

	//! Triangulate a Face
	void triangulate( Face* face );
	//! Triangulate the whole mesh
	void triangulate();
	
	class Extrusion {
	public:
		//! Sets the offset distance of the extrusion
		Extrusion& offset( float offset ) { mOffset = offset; return *this; }
		//! Sets the offset distance of the extrusion toward the center of the face
		Extrusion& innerOffset( float offset ) { mInnerOffset = offset; return *this; }
		//! Sets the direction of the extrusion
		Extrusion& direction( const glm::vec3 &dir ) { mDirection = dir; return *this; }
		//! Sets whether the .....
		Extrusion& cap( bool create = true ) { mCreateCap = create; return *this; }
		//! Sets the number of steps .....
		Extrusion& steps( uint16_t steps ) { mSteps = steps; return *this; }
		//! Sets whether the .....
		Extrusion& preserveGroups( bool preserve ) { mPreserveGroups = preserve; return *this; }

		Extrusion() 
		: mOffset( 0.0f ), mInnerOffset( 0.0f ), mDirection( 0.0f ), 
			mCreateCap( false ), mPreserveGroups( false ), mSteps( 1 ) {}
	protected:
		friend class Mesh;
		float mOffset;
		float mInnerOffset;
		ci::vec3 mDirection;
		bool mCreateCap;
		bool mPreserveGroups;
		uint16_t mSteps;
	};

	class Bevel {
	public:
	};
		
	//! Returns a pointer to the ith HalfEdge
	HalfEdge*	getHalfEdge( size_t i ) const;
	//! Returns a pointer to the ith Edge
	Edge*		getEdge( size_t i ) const;
	//! Returns a pointer to the ith Face
	Face*		getFace( size_t i ) const;
	//! Returns a pointer to the ith Boundary Face
	Face*		getBoundaryFace( size_t i ) const;
	//! Returns a pointer to the ith Vertex
	Vertex*		getVertex( size_t i ) const;

	//! Returns the number of HalfEdges
	size_t getNumHalfEdges() const;
	//! Returns the number of Edges
	size_t getNumEdges() const;
	//! Returns the number of Faces
	size_t getNumFaces() const;
	//! Returns the number of Boundary Faces
	size_t getNumBoundaryFaces() const;
	//! Returns the number of Vertices
	size_t getNumVertices() const;
		
	// Type aliases
	using HalfEdgePtr		= std::unique_ptr<class HalfEdge>;
	using HalfEdgeVector	= std::vector<HalfEdgePtr>;
	using EdgePtr			= std::unique_ptr<class Edge>;
	using EdgeVector		= std::vector<EdgePtr>;
	using VertexPtr			= std::unique_ptr<class Vertex>;
	using VertexVector		= std::vector<VertexPtr>;
	using FacePtr			= std::unique_ptr<class Face>;
	using FaceVector		= std::vector<FacePtr>;

	//! Returns the Mesh's HalfEdges
	const HalfEdgeVector&	getHalfEdges() const;
	//! Returns the Mesh's Edges
	const EdgeVector&		getEdges() const;
	//! Returns the Mesh's Faces
	const FaceVector&		getFaces() const;
	//! Returns the Mesh's Boundary Faces
	const FaceVector&		getBoundaryFaces() const;
	//! Returns the Mesh's Vertices
	const VertexVector&		getVertices() const;
	
	//! Checks and returns whether the mesh connectivity is complete
	bool isValid( bool verbose = false ) const;

	//! Returns whether the mesh is strictly composed of triangles
	bool isTriMesh() const;
	//! Returns whether the mesh is strictly composed of quads
	bool isQuadMesh() const;
	
	//! Creates and returns a VboMesh containing the Faces of the Mesh.
	ci::gl::VboMeshRef createFacesVbo() const;
	//! Creates and returns a VboMesh containing the Edges of the Mesh.
	ci::gl::VboMeshRef createEdgesVbo() const;
	//! Creates and returns a VboMesh containing the Points of the Mesh.
	ci::gl::VboMeshRef createPointsVbo() const;

	virtual ~Mesh();

protected:
	HalfEdge* makeHalfEdge();
	Edge* makeEdge();
	Face* makeFace();
	Face* makeBoundaryFace();
	Vertex* makeVertex( const ci::vec3 &position = ci::vec3( 0.0f ) );
	void eraseHalfEdge( HalfEdge* halfEdge );
	void eraseEdge( Edge* edge );
	void eraseFace( Face* face );
	void eraseBoundaryFace( Face* face );
	void eraseVertex( Vertex* vertex );

    void addFaces( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces );
	void copyFrom( const Mesh &mesh );

	bool calcPlaneIntersection( const HalfEdge* halfEdge, const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal, ci::vec3 *intersection, float *distance );
	
	friend class MeshSource;
	
	HalfEdgeVector	mHalfEdges;
	EdgeVector		mEdges;
	FaceVector		mFaces;
	FaceVector		mBoundaryFaces;
	VertexVector	mVertices;
};

class HalfEdge {
public:
	//! Returns the opposite HalfEdge
	HalfEdge*&	opposite() { return mOpposite; }
	//! Returns the const opposite HalfEdge
	HalfEdge*	opposite() const { return mOpposite; }
	//! Returns the next HalfEdge in this face
    HalfEdge*&	next() { return mNext; }
	//! Returns the const next HalfEdge in this face
    HalfEdge*	next() const { return mNext; }
	//! Returns the Edge this HalfEdge is part of
	Edge*&		edge() { return mEdge; }
	//! Returns the const Edge this HalfEdge is part of
    Edge*		edge() const { return mEdge; }
	//! Returns the Face this HalfEdge is part of
	Face*&		face() { return mFace; }
	//! Returns the const Face this HalfEdge is part of
	Face*		face() const { return mFace; }
	//! Returns the Vertex at the start of this HalfEdge
    Vertex*&	vertex() { return mVertex; }
	//! Returns the const Vertex at the start of this HalfEdge
    Vertex*		vertex() const { return mVertex; }
	
	//! Returns whether this HalfEdge belongs to a boundary face (shortcut to face()->isBoundary())
	bool isBoundary() const;
	//! Checks and returns whether the HalfEdge connectivity is valid
	bool isValid() const;

	HalfEdge() : mOpposite( nullptr ), mNext( nullptr ), mVertex( nullptr ), mEdge( nullptr ), mFace( nullptr ) {}
protected:
	HalfEdge*	mOpposite;
    HalfEdge*	mNext;
	Vertex*		mVertex;
    Edge*		mEdge;
	Face*		mFace;
};

class Face {
public:
	Face( bool boundary = false );

	//! Returns a HalfEdge associated with this Face
	HalfEdge*&	halfEdge() { return mHalfEdge; }
	//! Returns a const HalfEdge associated with this Face
	HalfEdge*	halfEdge() const { return mHalfEdge; }

	//! Returns the number of HalfEdges of this Face
	uint16_t	size() const;

	//! Calculates and returns the centroid point of the Face
	ci::vec3	calculateCentroid() const;
	//! Calculates and returns the average normal of the Face
	ci::vec3	calculateNormal() const;

	// http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
	//! TODO: Checks an returns whether the face is a convex polygon
	bool isConvex() const { return true; }
	//! Returns whether this Face is a boundary Face
	bool isBoundary() const { return mIsBoundary; }
	
	//! Checks and returns whether the Face connectivity is valid
	bool isValid() const;

	//! Calculates and returns the face Axis Aligned Bounding Box
	ci::AxisAlignedBox calculateBounds() const;
	//! Returns whether there is an intersection between the ray and the face
	bool intersect( const ci::Ray &r, float* dist ) const;
	
	//! Returns whether this face is selected
	bool isSelected() const;
			 
protected:
	bool		mIsSelected;
	bool		mIsBoundary;
	HalfEdge*	mHalfEdge;
};

class Vertex {
public:
	Vertex( const ci::vec3 &position ) : mPosition( position ), mHalfEdge( nullptr ) {}

	//! Returns a HalfEdge associated with this Vertex
	HalfEdge*&	halfEdge() { return mHalfEdge; }
	//! Returns a const HalfEdge associated with this Vertex
	HalfEdge*	halfEdge() const { return mHalfEdge; }

	uint16_t	degree() const;
	
	//! Returns the Vertex position
	ci::vec3	getPosition() const { return mPosition; }
	//! Sets the Vertex position
	void		setPosition( const ci::vec3 &position ) { mPosition = position; }
    
	//! Checks and returns whether the Vertex connectivity is valid
	bool isValid() const;

protected:
	ci::vec3	mPosition;
	HalfEdge*	mHalfEdge;
};

class Edge {
public:

	//! Returns a HalfEdge associated with this Edge
	HalfEdge*&	halfEdge() { return mHalfEdge; }
	//! Returns a const HalfEdge associated with this Edge
	HalfEdge*	halfEdge() const { return mHalfEdge; }
	
	//! Checks and returns whether the Edge connectivity is valid
	bool isValid() const;

	Edge() : mHalfEdge( nullptr ) {}
protected:
	HalfEdge* mHalfEdge;
};

class MeshSource {
public:
	virtual void loadInto( Mesh *target ) const = 0;
	virtual ~MeshSource() {}
protected:
    void addFaces( Mesh* mesh, const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces ) const { mesh->addFaces( vertices, faces ); }
};

class Exception : public ci::Exception {
public:
	Exception( const std::string &message ) : ci::Exception( message ) {}
};

} // namespace halfedge

namespace he = halfedge;