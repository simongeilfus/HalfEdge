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

#include "halfedge/Mesh.h"

#include "cinder/log.h"
#include "cinder/TriMesh.h"
#include "cinder/gl/VboMesh.h"


// CSG
// http://sandervanrossen.blogspot.be/2009/12/realtime-csg-part-1.html

// halfedge
// https://fgiesen.wordpress.com/2012/02/21/half-edge-based-mesh-representations-theory/
// http://www.flipcode.com/archives/The_Half-Edge_Data_Structure.shtml
// http://kaba.hilvi.org/homepage/blog/halfedge/halfedge.htm

// other impl
// https://github.com/zzhan133/Subdivision
// https://github.com/atrojas/halfedge_mesh (py)
// https://github.com/prideout/aobaker/blob/e65011a88519ec473a5d4955522cedc571fff4a0/thekla/nvmesh/param/Util.cpp
// https://github.com/462cmu/asst2_meshedit/blob/master/src/halfEdgeMesh.h
// https://github.com/abedchehab/ComputerGraphics/blob/7e5c183d6d31d210ddd49cd78808f9637c45913d/MeshViewer/myproj/myMesh.cpp
// https://github.com/weshoke/ofxHEMesh/blob/master/src/ofxHEMesh.h
	
// Random
// https://github.com/zeux/meshoptimizer
// https://mattdesl.svbtle.com/drawing-lines-is-hard

// faces normals
// http://www.bytehazard.com/articles/vertnorm.html
// http://www.codeguru.com/cpp/g-m/opengl/article.php/c2681/Computing-normals-to-achieve-flat-and-smooth-shading.htm

// triangle strips
// http://www.codercorner.com/Strips.htm
// http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.409.3206&rep=rep1&type=pdf

using namespace std;
using namespace ci;

namespace halfedge {

MeshRef Mesh::create()
{
	return std::make_shared<Mesh>();
}

MeshRef Mesh::create( const Mesh &mesh )
{
	return std::make_shared<Mesh>( mesh );
}

MeshRef Mesh::create( const MeshSource &source )
{
	return std::make_shared<Mesh>( source );
}

MeshRef Mesh::create( const std::vector<ci::vec3> &vertices, const std::vector<uint32_t> &indices )
{
	return std::make_shared<Mesh>( vertices, indices );
}

MeshRef Mesh::create( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces )
{
	return std::make_shared<Mesh>( vertices, faces );
}

MeshRef Mesh::create( const ci::TriMesh &trimesh )
{
	return std::make_shared<Mesh>( trimesh );
}

Mesh::Mesh()
{
}

Mesh::Mesh( const Mesh &mesh )
: Mesh()
{
	copyFrom( mesh );
}

Mesh::Mesh( const MeshSource &source )
: Mesh()
{
	source.loadInto( this );
}

Mesh::Mesh( const std::vector<ci::vec3> &vertices, const std::vector<uint32_t> &indices )
: Mesh()
{
	addFaces( vertices, { indices } );
}

Mesh::Mesh( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces )
: Mesh()
{
	addFaces( vertices, faces );
}

Mesh::Mesh( const ci::TriMesh &trimesh )
: Mesh()
{
	auto trimeshIndices = trimesh.getIndices();
	std::vector<std::vector<uint32_t>> indices;
	for( int i = 0; i < trimeshIndices.size(); i+=3 ) {
		indices.push_back( { trimeshIndices[i], trimeshIndices[i+1], trimeshIndices[i+2] } );
	}
	auto trimeshPositions = trimesh.getPositions<3>();
	std::vector<ci::vec3> positions;
	for( int i = 0; i < trimesh.getNumVertices(); ++i ) {
		positions.push_back( trimeshPositions[i] );
	}

	addFaces( positions, indices );
}

Mesh::~Mesh()
{
}

const Mesh& Mesh::operator=( const Mesh &mesh )
{
	copyFrom( mesh );
	return *this;
}
void Mesh::copyFrom( const Mesh &mesh )
{
	CI_LOG_I( "!Deep copy!" );
	// clear vectors
	mHalfEdges.clear();
	mEdges.clear();
	mFaces.clear();
	mBoundaryFaces.clear();
	mVertices.clear();
	
	// temporarly store the input mesh connectivity data to the new data
	std::map<HalfEdge*,HalfEdge*>	halfEdgeLinks;
	std::map<Edge*,Edge*>			edgeLinks;
	std::map<Face*,Face*>			faceLinks;
	std::map<Vertex*,Vertex*>		vertexLinks;
	
	// create new data and temporarly copy all the old pointers
	for( const auto &halfEdge : mesh.getHalfEdges() ) {
		auto newHalfEdge = makeHalfEdge();
		newHalfEdge->next() = halfEdge->next();
		newHalfEdge->opposite() = halfEdge->opposite();
		newHalfEdge->face() = halfEdge->face();
		newHalfEdge->vertex() = halfEdge->vertex();
		newHalfEdge->edge() = halfEdge->edge();
		halfEdgeLinks[halfEdge.get()] = newHalfEdge;
	}
	for( const auto &edge : mesh.getEdges() ) {
		auto newEdge = makeEdge();
		newEdge->halfEdge() = edge->halfEdge();
		edgeLinks[edge.get()] = newEdge;
	}
	for( const auto &face : mesh.getFaces() ) {
		auto newFace = makeFace();
		newFace->halfEdge() = face->halfEdge();
		faceLinks[face.get()] = newFace;
	}
	for( const auto &vertex : mesh.getVertices() ) {
		auto newVertex = makeVertex( vertex->getPosition() );
		newVertex->halfEdge() = vertex->halfEdge();
		vertexLinks[vertex.get()] = newVertex;
	}
	for( const auto &face : mesh.getBoundaryFaces() ) {
		auto newFace = makeBoundaryFace();
		newFace->halfEdge() = face->halfEdge();
		faceLinks[face.get()] = newFace;
	}

	// update connectivity of the new data by replacing the old pointers by the new ones
	for( const auto &halfEdge : mHalfEdges ) {
		halfEdge->next() = halfEdgeLinks[halfEdge->next()];
		halfEdge->opposite() = halfEdgeLinks[halfEdge->opposite()];
		halfEdge->face() = faceLinks[halfEdge->face()];
		halfEdge->vertex() = vertexLinks[halfEdge->vertex()];
		halfEdge->edge() = edgeLinks[halfEdge->edge()];
	}
	for( const auto &edge : mEdges ) {
		edge->halfEdge() = halfEdgeLinks[edge->halfEdge()];
	}
	for( const auto &face : mFaces ) {
		face->halfEdge() = halfEdgeLinks[face->halfEdge()];
	}
	for( const auto &vertex : mVertices ) {
		vertex->halfEdge() = halfEdgeLinks[vertex->halfEdge()];		
	}
	for( const auto &face : mBoundaryFaces ) {
		face->halfEdge() = halfEdgeLinks[face->halfEdge()];		
	}
}


void Mesh::addFaces( const std::vector<ci::vec3> &vertices, const std::vector<std::vector<uint32_t>> &faces )
{
	// temp structures
	std::map<Vertex*,uint32_t> verticesDegrees;
	std::map<uint32_t,Vertex*> indicesVertices;
	using DirectedEdge = std::pair<uint32_t,uint32_t>;
	std::map<DirectedEdge,HalfEdge*> directedEdgesHalfEdges;
	//auto boundaryMarker = std::make_unique<HalfEdge>();
      
	// check if the input faces are valid
	for( const auto &face : faces )	{
		// faces need at least 3 vertices
		if( face.size() < 3 ) {
			throw Exception( "A Face in the input Mesh has less than 3 vertices" );
		}
   
		// allocate vertices and calculate the vertices degrees to check for non-manifold mesh
		std::set<uint32_t> faceIndices;
		for( const auto &indice : face ) {
			faceIndices.insert( indice );
   
			// if the vertex is already in used increment its degree
			if( indicesVertices.count( indice ) ) {
				verticesDegrees[indicesVertices[indice]]++;
			}
			// otherwise create a new vertex
			else {
				Vertex* vertex = makeVertex( vertices[indice] );
				vertex->halfEdge()		= nullptr;//boundaryMarker.get();
				verticesDegrees[vertex] = 1;
				indicesVertices[indice]	= vertex;
			}
		}
   
		// check if this face uses a vertex several times
		if( faceIndices.size() < face.size() ) {
			throw Exception( "A Face in the input Mesh use the same vertex several times" );
		} 
	}

	// create Faces and new HalfEdges
	for( const auto &face : faces )	{
		// iterate over each directed edges of the face
		size_t numVertices = face.size();
		std::vector<HalfEdge*> halfEdges;
		Face* newFace = nullptr;
		for( size_t i = 0; i < numVertices; ++i ) {			
			DirectedEdge directedEdge( face[i], face[(i+1)%numVertices] );
			HalfEdge* halfEdge;

			// check if the directed edge exist already
			if( directedEdgesHalfEdges.count( directedEdge ) ) {
				throw Exception( "The input Mesh has duplicated directed edges which means that the surface is non-manifold or has orientation inconsistencies." );
			}
			// otherwise create a new HalfEdge
			else {
				// and a new Face if needed
				if( ! newFace ) {
					newFace = makeFace();
				}
				halfEdge						= makeHalfEdge();
				halfEdge->face()				= newFace;
				newFace->halfEdge()				= halfEdge;
				halfEdge->face()->halfEdge()	= halfEdge;
				halfEdge->vertex()				= indicesVertices[directedEdge.first];
				halfEdge->vertex()->halfEdge()	= halfEdge;

				directedEdgesHalfEdges[directedEdge] = halfEdge;
				halfEdges.push_back( halfEdge );
			}

			// check if the opposite HalfEdge already exists
			DirectedEdge oppositeDirectedEdge( directedEdge.second, directedEdge.first );
			if( directedEdgesHalfEdges.count( oppositeDirectedEdge ) ) {
				// create a new Edge
				Edge* edge = makeEdge();
				edge->halfEdge() = halfEdge;

				// and update the connections	
				HalfEdge* opposite = directedEdgesHalfEdges[oppositeDirectedEdge];
				opposite->opposite() = halfEdge;
				opposite->edge() = edge;
				halfEdge->opposite() = opposite;
				halfEdge->edge() = edge;
			}
			else {
				// some HalfEdge opposite will remain null which means
				// they lie on the boundary of the mesh
				halfEdge->opposite() = nullptr;//boundaryMarker.get();
			}
		}
		
		// finish the face loop by setting each HalfEdge next pointer
		for( size_t i = 0; i < numVertices; ++i ) {
			halfEdges[i]->next() = halfEdges[(i+1)%numVertices];
		}
	}
   
	// set boundary vertices HalfEdge pointer to a boundary HalfEdge
	for( const auto &vertex : mVertices ) {
		auto halfEdge = vertex->halfEdge();
		do {
			if( halfEdge->opposite() == nullptr ) {//boundaryMarker.get() ) {
				vertex->halfEdge() = halfEdge;
				break;
			}
   
			halfEdge = halfEdge->opposite()->next();
		} while( halfEdge != vertex->halfEdge() );
	}

	// create the boundary faces
	size_t numHalfEdges = mHalfEdges.size();
	for( size_t i = 0; i < numHalfEdges; ++i ) {
		HalfEdge* halfEdge = mHalfEdges[i].get();
		
		// if the HalfEdge lie on the boundary (marked in the previous loop)
		if( halfEdge->opposite() == nullptr ) {//boundaryMarker.get() ) {
			// create the boundary Face
			auto boundaryFace = makeBoundaryFace();

			// iterate over the boundary 
			std::vector<HalfEdge*> halfEdges;
			HalfEdge* it = halfEdge;
			do {
				// and create the boundary face half edge and shared edge
				auto opposite			= makeHalfEdge();
				auto edge				= makeEdge();
				it->opposite()			= opposite;
				it->edge()				= edge;
				opposite->opposite()	= it;
				opposite->face()		= boundaryFace;
				opposite->vertex()		= it->next()->vertex();
				opposite->edge()		= edge;
				edge->halfEdge()		= it;
				halfEdges.push_back( opposite );

   				it = it->next();
				
				// find the next boundary HalfEdge by checking every halfEdge connected to the next vertex
				while( it != halfEdge && it->opposite() != nullptr ) {//boundaryMarker.get() ) {
					it = it->opposite()->next();
				}
			} while( it != halfEdge );
   
			// finish the boundary face loop by setting each HalfEdge next pointer in reversed order
			size_t numVertices = halfEdges.size();
			for( size_t i = 0; i < numVertices; ++i ) {
				halfEdges[i]->next() = halfEdges[(i-1+numVertices)%numVertices];
			}   
			boundaryFace->halfEdge() = halfEdges[0];
		}   
	}
   
	// reverse vertices HalfEdge connectivity
	for( const auto &vertex : mVertices ) {
		vertex->halfEdge() = vertex->halfEdge()->opposite()->next();
	}
      
	// check for errors
	for( const auto &vertex : mVertices ) {
		// check if the vertex is connected to the mesh
		if( vertex->halfEdge() == nullptr ) {
			throw Exception( "The input Mesh has a vertex that doesn't belong to any face" );
		}
   
		// check for "manifoldness"
		size_t count = 0;
		auto halfEdge = vertex->halfEdge();
		do {
			if( ! halfEdge->face()->isBoundary() ) {
				count++;
			}
			halfEdge = halfEdge->opposite()->next();
		} while( halfEdge != vertex->halfEdge() );
   
		if( count != verticesDegrees[vertex.get()] ) {
			throw Exception( "The input Mesh has a nonmanifold vertex" );
		}
	}
   
	// at this stage if the number of input vertices differ from the mesh's vertices that means that there is duplicated or missing vertices
	if( vertices.size() != mVertices.size() ) {
		throw Exception( "The input Mesh has duplicated or missing vertices" );
	}
}


Edge* Mesh::splitFace( Face* face, Vertex* a, Vertex* b )
{	
	// check if face has more than 3 vertices
	if( face->size() < 4 ) {
		throw Exception( "Trying to split a face with less than 4 vertices." );
	}
	// check if b belongs to the face and save the halfedge
	auto abNext = b->halfEdge();
	do {
		if( abNext->face() == face ) {
			break;
		}
		abNext = abNext->opposite()->next();
	} while( abNext != b->halfEdge() );

	if( abNext->face() != face ) {
		throw Exception( "Vertex A doesn't belong to the Face you're trying to split." );
	}
	
	// check if a belongs to the face and save the halfedge
	auto baNext = a->halfEdge();
	do {
		if( baNext->face() == face ) {
			break;
		}
		baNext = baNext->opposite()->next();
	} while( baNext != a->halfEdge() );

	if( baNext->face() != face ) {
		throw Exception( "Vertex B doesn't belong to the Face you're trying to split." );
	}

	// create the edge and halfedges
	auto edge = makeEdge();
	auto ab = makeHalfEdge();
	auto ba = makeHalfEdge();

	// connect them together and to the two vertices
	edge->halfEdge()	= ab;
	ab->opposite()		= ba;
	ab->vertex()		= a;
	ab->edge()			= edge;
	ab->next()			= abNext;
	ba->opposite()		= ab;
	ba->vertex()		= b;
	ba->edge()			= edge;
	ba->next()			= baNext;
	
	// start at abNext and run around the face until abPrev
	auto abPrev = abNext;
	while( abPrev->next() != baNext ) {
		abPrev->face() = face;
		abPrev = abPrev->next();
	}

	// connect ab to the face and its previous halfedge
	abPrev->next() = ab;
	ab->face() = face;
	ab->face()->halfEdge() = ab;
	
	// create a new face and connect it to ba
	auto newFace = makeFace();
	ba->face() = newFace;
	ba->face()->halfEdge() = ba;

	// continue looping from baNext to find baPrev and update
	// the face connections along the way
	auto baPrev = baNext;
	while( baPrev->next() != abNext ) {
		baPrev->face() = newFace;
		baPrev = baPrev->next();
	}
	baPrev->next() = ba;
	baPrev->face() = newFace;

	return edge;
}

bool Mesh::calcPlaneIntersection( const HalfEdge* halfEdge, const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal, ci::vec3 *intersection, float *distance )
{
	ci::vec3 start = halfEdge->vertex()->getPosition();
	ci::vec3 end = halfEdge->opposite()->vertex()->getPosition();
	ci::vec3 direction = end - start;
	float length = glm::length( direction );
	direction /= length;
	float denom = glm::dot( planeNormal, direction );
	if( glm::abs( denom ) > 0.0000001f ) {
		*distance = glm::dot( planeNormal, planeOrigin - start ) / denom + 0.00001f;
		if( *distance > 0.0f && *distance <= length ) {
			*intersection = start + direction * *distance;
			return true;
		}
	}
	return false;
}
	
Edge* Mesh::splitFace( Face* face, const ci::Planef &plane )
{
	return splitFace( face, plane.getPoint(), plane.getNormal() );
}

Edge* Mesh::splitFace( Face* face, const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal )
{
	const float eps = 0.0000001f;
	bool firstIntersection = false;
	ci::vec3 firstIntersectionPoint;
	float firstIntersectionDistance;
	auto halfEdge = face->halfEdge();
	auto intersectingHalfEdge = halfEdge;
	do {
		float distance;
		ci::vec3 intersection;
		if( calcPlaneIntersection( halfEdge, planeOrigin, planeNormal, &intersection, &distance ) ) {
			if( firstIntersection ) {
				auto vert0 = firstIntersectionDistance < eps ? intersectingHalfEdge->vertex() : splitEdge( intersectingHalfEdge->edge() );
				auto vert1 = distance < eps ? halfEdge->vertex() : splitEdge( halfEdge->edge() );
				vert0->setPosition( firstIntersectionPoint );
				vert1->setPosition( intersection );
				return splitFace( face, vert1, vert0 );
			}
			firstIntersectionPoint = intersection;
			firstIntersectionDistance = distance;
			firstIntersection = true;
			intersectingHalfEdge = halfEdge;
		}
		
		halfEdge = halfEdge->next();
	} while( halfEdge != face->halfEdge() );

	return nullptr;
}

Vertex* Mesh::splitFace( Face* face )
{
	// split the first vertex and put it at the center of the face
	auto centroid = face->calculateCentroid();
	auto vertex	= splitVertex( face->halfEdge()->vertex(), face );
	vertex->setPosition( centroid );

	// connect remaining vertices to the new vertex
	auto halfEdge = face->halfEdge()->next();
	while( halfEdge->next() != face->halfEdge() ) {
		splitFace( face, vertex, halfEdge->vertex() );
		halfEdge = halfEdge->next();
	}

	return vertex;
}

void Mesh::splitFaces( const ci::Planef &plane )
{
	splitFaces( plane.getPoint(), plane.getNormal() );
}

void Mesh::splitFaces( const ci::vec3 &planeOrigin, const ci::vec3 &planeNormal )
{
	// try to split all faces
	size_t numFaces = mFaces.size();
	for( size_t i = 0; i < numFaces; ++i ){
		splitFace( mFaces[i].get(), planeOrigin, planeNormal );
	}
}

Vertex* Mesh::splitEdge( Edge* edge, float t )
{
	// get the halfedges
	auto ab = edge->halfEdge();
	auto ba = edge->halfEdge()->opposite();
	auto baPrev = ba->next();
	while( baPrev->next() != ba ) {
		baPrev = baPrev->next();
	}

	// create the new vertex, edge and halfedges
	auto vertex = makeVertex( glm::mix( ab->vertex()->getPosition(), ba->vertex()->getPosition(), t ) );
	auto newEdge = makeEdge();
	auto newAb = makeHalfEdge();
	auto newBa = makeHalfEdge();
	
	// connect them together and to the two vertices
	vertex->halfEdge()	= newAb;
	newEdge->halfEdge()	= newAb;
	newAb->edge()		= newEdge;
	newBa->edge()		= newEdge;
	newAb->opposite()	= newBa;
	newBa->opposite()	= newAb;
	newAb->face()		= ab->face();
	newBa->face()		= ba->face();
	newAb->vertex()		= vertex;
	newBa->vertex()		= ba->vertex();

	ba->vertex()->halfEdge()	= newBa;
	ba->vertex()				= vertex;

	// update next connectivity
	newAb->next()	= ab->next();
	ab->next()		= newAb;
	newBa->next()	= ba;
	baPrev->next()	= newBa;

	return vertex;
}

Vertex* Mesh::splitEdge( Edge* edge, const ci::vec3 &splitPosition )
{
	auto vert = splitEdge( edge );
	vert->setPosition( splitPosition );
	return vert;
}

Vertex* Mesh::splitVertex( Vertex* vertex, Face* face )
{
	// create edge, half edges and vertex
	auto edge = makeEdge();
	auto ab = makeHalfEdge();
	auto ba = makeHalfEdge();
	auto newVertex = makeVertex( ci::vec3( 0.0f ) );

	// link things together
	edge->halfEdge() = ab;
	newVertex->halfEdge() = ba;

	ab->opposite() = ba;
	ab->face() = face;
	ab->edge() = edge;
	ab->vertex() = vertex;
	ab->next() = ba;

	ba->opposite() = ab;
	ba->face() = face;
	ba->edge() = edge;
	ba->vertex() = newVertex;

	// find ba next by looping around the vertex until finding
	// an halfedge that belongs to the face
	ba->next() = vertex->halfEdge();
	do {
		if( ba->next()->face() == face ) {
			break;
		}
		ba->next() = ba->next()->opposite()->next();
	} while( ba->next() != vertex->halfEdge() );

	if( ba->next()->face() != face ) {
		throw Exception( "Trying to split a vertex that doesn't belong to this face" );
	}

	// find abprev by continue looping until the vertex is found again
	auto abPrev = ba->next();
	while( abPrev->next()->vertex() != vertex ) {
		abPrev = abPrev->next();
	}
	abPrev->next() = ab;

	return newVertex;
}

Vertex* Mesh::splitVertex( Vertex* vertex, HalfEdge* firstHalfEdge, HalfEdge* lastHalfEdge )
{
	if( firstHalfEdge->vertex() != vertex ) {
		throw Exception( "HalfEdge* firstHalfEdge is not connected to input vertex." );	
	}
	if( lastHalfEdge != nullptr && lastHalfEdge->vertex() != vertex ) {
		throw Exception( "HalfEdge* lastHalfEdge is not connected to input vertex." );
	}
	else if( lastHalfEdge == nullptr ) {
		lastHalfEdge = firstHalfEdge;
	}
	
	// create the new vertex, edge and halfedges
	auto newVertex = makeVertex( vertex->getPosition() );
	auto edge = makeEdge();
	auto ab = makeHalfEdge();
	auto ba = makeHalfEdge();

	// the starting edge becomes AB next
	auto abNext = firstHalfEdge;
	// and the ending edge BA prev
	auto baPrev = lastHalfEdge->opposite();
	// BA next should be the current BA prev next
	auto baNext = baPrev->next();
	// find AB prev
	auto abPrev = baNext->opposite();
	while( abPrev->next() != abNext ) {
		abPrev = abPrev->next()->opposite();
	}
	
	// connect the new vertex to the moving halfEdges
	for( auto halfEdge = firstHalfEdge; halfEdge != baNext; halfEdge = halfEdge->opposite()->next() ) {
		halfEdge->vertex() = newVertex;

	}
	// connect everything else
	edge->halfEdge()		= ab;
	
	ab->edge()				= edge;
	ab->opposite()			= ba;
	ab->face()				= abNext->face();
	ab->next()				= abNext;
	ab->vertex()			= vertex;
	abPrev->next()			= ab;
	
	ba->edge()				= edge;
	ba->opposite()			= ab;
	ba->face()				= baNext->face();
	ba->next()				= baNext;
	ba->vertex()			= newVertex;
	baPrev->next()			= ba;

	newVertex->halfEdge()	= ba;
	vertex->halfEdge()		= ab;
	
	return newVertex;
}

Vertex* Mesh::splitVertex( Vertex* vertex, Edge* firstEdge, Edge* lastEdge )
{
	if( firstEdge->halfEdge()->vertex() != vertex 
		&& firstEdge->halfEdge()->opposite()->vertex() != vertex ) {
		throw Exception( "Edge* firstEdge is not connected to input vertex." );	
	}
	if( lastEdge != nullptr && lastEdge->halfEdge()->vertex() != vertex 
		&& lastEdge->halfEdge()->opposite()->vertex() != vertex ) {
		throw Exception( "Edge* lastEdge is not connected to input vertex." );
	}
	
	auto firstHalfEdge = firstEdge->halfEdge()->vertex() == vertex ? firstEdge->halfEdge() : firstEdge->halfEdge()->opposite();
	auto lastHalfEdge = lastEdge == nullptr ? firstHalfEdge : lastEdge->halfEdge()->vertex() == vertex ? lastEdge->halfEdge() : lastEdge->halfEdge()->opposite();
	return splitVertex( vertex, firstHalfEdge, lastHalfEdge );
}

Vertex* Mesh::collapseEdge( Edge* edge, float t )
{
	// check if the operation is possible (if both faces have more than 3 edges)
	if( ( ! edge->halfEdge()->face()->isBoundary() && edge->halfEdge()->face()->size() <= 3 ) 
		|| ( ! edge->halfEdge()->opposite()->face()->isBoundary() && edge->halfEdge()->opposite()->face()->size() <= 3 ) ) {
		throw Exception( "The two faces bordering an edge need more than 3 sides for an edge collapse." );
	}

	// get the halfedges
	auto ab = edge->halfEdge();
	auto ba = edge->halfEdge()->opposite();
	auto abNext = ab->next();
	auto baNext = ba->next();
	
	auto abPrev = ab->next();
	while( abPrev->next() != ab ) {
		abPrev = abPrev->next();
	}

	auto baPrev = ba->next();
	while( baPrev->next() != ba ) {
		baPrev = baPrev->next();
	}

	// keep one vertex and move it to its new position
	auto vertex = ab->vertex();
	vertex->setPosition( glm::mix( vertex->getPosition(), ba->vertex()->getPosition(), t ) );
	vertex->halfEdge() = abNext;
	
	// update connectivity
	abPrev->next() = abNext;
	baPrev->next() = baNext;
	abPrev->face()->halfEdge() = abPrev;
	baPrev->face()->halfEdge() = baPrev;
	abNext->vertex() = vertex;
	baNext->vertex() = vertex;

	// update halfedges starting at b vertices
	auto it = ba->opposite()->next();
	do {
		it->vertex() = vertex;
		it = it->opposite()->next();
	} while( it != ba->opposite()->next() );

	// remove the edge, the two halfedges and the remaining vertex
	eraseEdge( edge );
	eraseVertex( ba->vertex() );
	eraseHalfEdge( ab );
	eraseHalfEdge( ba );

	return vertex;
}

Vertex* Mesh::collapseEdge( Vertex* vertex0, Vertex* vertex1, float t )
{
	Edge* edge = nullptr;
	auto it = vertex0->halfEdge();
	do {
		if( it->next()->vertex() == vertex1 ) {
			edge = it->edge();
		}
	
	} while( it != vertex0->halfEdge() );

	if( ! edge ) {
		throw Exception( "Trying to collapse two vertices that doesn't share an Edge." );
	}

	return collapseEdge( edge );
}

Vertex* Mesh::collapseHalfEdge( HalfEdge* halfEdge, float t )
{
	return collapseEdge( halfEdge->edge(), halfEdge->edge()->halfEdge() == halfEdge ? t : 1.0f - t );
}

void Mesh::extrude( Face* face, const Extrusion &extrusion )
{
	float innerOffset = extrusion.mInnerOffset / static_cast<float>( extrusion.mSteps );
	float offset = extrusion.mOffset / static_cast<float>( extrusion.mSteps );
	auto normal = glm::any( glm::notEqual( ci::vec3( 0.0f ), extrusion.mDirection ) ) ? glm::normalize( extrusion.mDirection ) : face->calculateNormal();
	auto centroid = face->calculateCentroid();
	auto baseVertex = face->halfEdge()->vertex();
	for( size_t i = 0; i < extrusion.mSteps; ++i ) {

		// start by getting the list of vertices to split
		std::vector<Vertex*> verticesToSplit;
		{
			auto it = face->halfEdge();
			do {
				verticesToSplit.push_back( it->vertex() );
				it = it->next();
			} while( it != face->halfEdge() );
		}
	
		// split the vertices into floating vertices
		std::vector<Vertex*> newVertices;
		for( auto vertToSplit : verticesToSplit ) {
			auto vert = splitVertex( vertToSplit, face );
			auto toCentroid = glm::normalize( centroid - vertToSplit->getPosition() );
			vert->setPosition( vertToSplit->getPosition() + toCentroid * innerOffset + normal * offset );
			newVertices.push_back( vert );
		}

		// use the new vertices to split the face
		for( size_t i = 0; i < newVertices.size(); ++i ) {
			splitFace( face, newVertices[i], newVertices[(i+1)%newVertices.size()] );
		}
	}
		
	// see if we need and can create the cap
	auto it = baseVertex->halfEdge()->face()->isBoundary() ? baseVertex->halfEdge() : baseVertex->halfEdge()->opposite();
	if( extrusion.mCreateCap && it->face()->isBoundary() ) {
		auto end = it;
		auto boundary = it->face();
		auto cap = makeFace();
		cap->halfEdge() = it;
		do {
			it->face() = cap;
			it = it->next();
		} while( it != end );

		eraseBoundaryFace( boundary );
	}
}
void Mesh::extrude( Face* face, float offset )
{
	extrude( face, Extrusion().offset( offset ) );
}
void Mesh::extrudeInner( Face* face, float offset )
{
	extrude( face, Extrusion().innerOffset( offset ) );
}
void Mesh::extrude( const std::vector<Face*> &faces, const Extrusion &extrusion )
{
	float innerOffset = extrusion.mInnerOffset / static_cast<float>( extrusion.mSteps );
	float offset = extrusion.mOffset / static_cast<float>( extrusion.mSteps );	
	// Note: Those maps are very costy and probably fragment the memory a lot
	std::map<Vertex*,std::vector<Face*>> verticesSharedFaces;
	std::map<Vertex*,bool> extrudedVertices;
	std::map<Edge*,std::vector<Face*>> edgesSharedFaces;
	std::map<Face*,ci::vec3> facesCentroids, facesNormals;
	for( size_t i = 0; i < extrusion.mSteps; ++i ) {
		// create helper structures
		// Note: Those maps are very costy and probably fragment the memory a lot
		verticesSharedFaces.clear();
		extrudedVertices.clear();
		edgesSharedFaces.clear();
		facesCentroids.clear();
		facesNormals.clear();
		for( const auto &face : faces ) {
			auto it = face->halfEdge();
			do {
				verticesSharedFaces[it->vertex()].push_back( face );
				edgesSharedFaces[it->edge()].push_back( face );
				extrudedVertices[it->vertex()] = false;
				it = it->next();
			} while( it != face->halfEdge() );
			facesCentroids[face] = face->calculateCentroid();
			facesNormals[face] = face->calculateNormal();
		}

		// count the number of boundary edges and keep the last one as the starting point
		size_t numEdges = 0;
		std::vector<Edge*> boundaryEdges;
		for( const auto &edge : edgesSharedFaces ) {
			if( edge.second.size() == 1 ) {
				boundaryEdges.push_back( edge.first );
				numEdges++;
			}
		}
		
		while( ! boundaryEdges.empty() ) {
			auto edge = boundaryEdges.back();
			// the edge HalfEdge that is not on one of our faces is the first border HalfEdge
			auto halfEdge = edge->halfEdge()->face() != edgesSharedFaces[edge][0] ? edge->halfEdge() : edge->halfEdge()->opposite();
			auto baseHalfEdge = halfEdge;
		
			// split any vertex that need to be split
			std::vector<Vertex*> splits;
			do {
				// find next halfEdge on the border
				auto next = halfEdge->next();
				do {
					if( edgesSharedFaces.count( next->edge() ) && edgesSharedFaces[next->edge()].size() == 1 ) {
						break;
					}
					next = next->opposite()->next();
				} while( next != halfEdge );

				// corner vertex split
				size_t numSharedFaces = verticesSharedFaces[halfEdge->vertex()].size();
				if( numSharedFaces == 1 ) {
					auto toCentroid = glm::normalize( facesCentroids[halfEdge->opposite()->face()] - halfEdge->vertex()->getPosition() );
					auto normal = glm::any( glm::notEqual( ci::vec3( 0.0f ), extrusion.mDirection ) ) ? glm::normalize( extrusion.mDirection ) : facesNormals[halfEdge->opposite()->face()];
					auto vert = splitVertex( halfEdge->vertex(), halfEdge->opposite()->face() );
					vert->setPosition( halfEdge->vertex()->getPosition() + toCentroid * innerOffset + normal * offset );
					extrudedVertices[vert] = true;
					splits.push_back( vert );
				}
				// split along shared edge
				else if( numSharedFaces == 2 ) {
					// walk around the vertex and see if the edges in between need to be splitted
					auto it = halfEdge;
					do {
						if( facesCentroids.count( it->face() ) && ( ! edgesSharedFaces.count( it->edge() ) || edgesSharedFaces[it->edge()].size() != 1 ) ) {
							auto toNext = glm::normalize( it->next()->vertex()->getPosition() - it->vertex()->getPosition() );
							auto normal = glm::any( glm::notEqual( ci::vec3( 0.0f ), extrusion.mDirection ) ) ? glm::normalize( extrusion.mDirection ) : facesNormals[it->face()];
							auto vertex = it->vertex();
							auto newVertex = splitVertex( vertex, it );
							newVertex->setPosition( vertex->getPosition() + toNext * innerOffset * glm::sin( (float) M_PI * 0.25f ) + normal * offset );
							extrudedVertices[newVertex] = true;
							splits.push_back( newVertex );
							it = it->opposite()->next()->next();
						}
						else {
							it = it->opposite()->next();
						}
					} while( it != halfEdge );
				}
				else if( numSharedFaces > 2 ) {
					// get the first and last edge inside the selected faces
					auto it = halfEdge;
					HalfEdge* first = nullptr;
					HalfEdge* last = nullptr;
					do {
						if( facesCentroids.count( it->face() ) && ( ! edgesSharedFaces.count( it->edge() ) || edgesSharedFaces[it->edge()].size() != 1 ) ) {
							if( first == nullptr ) {
								first = it;
							}
							else {
								last = it;
							}
						}	
						it = it->opposite()->next();			
					} while( it != halfEdge );
			
					auto toCentroid = glm::normalize( facesCentroids[last->face()] - halfEdge->vertex()->getPosition() );
					auto normal = glm::any( glm::notEqual( ci::vec3( 0.0f ), extrusion.mDirection ) ) ? glm::normalize( extrusion.mDirection ) : facesNormals[last->face()];
					auto vertex = splitVertex( first->vertex(), first, last );
					vertex->setPosition( halfEdge->vertex()->getPosition() + toCentroid * innerOffset + normal * offset );
					extrudedVertices[vertex] = true;
					splits.push_back( vertex );
				}

				// remove the edge from the processing list
				auto it = std::find_if( boundaryEdges.begin(), boundaryEdges.end(), [halfEdge]( Edge* edge ) {
					return edge == halfEdge->edge();
				} );
				if( it != boundaryEdges.end() ) {
					boundaryEdges.erase( it );
				}

				// advance on the border
				halfEdge = next;
			} while( halfEdge != baseHalfEdge );
		
			// split the faces with the resulting vertices
			size_t numSplits = splits.size();
			for( size_t i = 0; i < numSplits; ++i ) {
				auto vert0 = splits[i];
				auto vert1 = splits[(i+1)%numSplits];
				// find the shared face
				Face* face = nullptr;
				auto it0 = vert0->halfEdge();
				do {
					auto it1 = vert1->halfEdge();
					do {
						if( it1->face() == it0->face() ) {
							face = it1->face();
							break;
						}
						it1 = it1->opposite()->next();
					} while( it1 != vert1->halfEdge() );
					it0 = it0->opposite()->next();
				} while( it0 != vert0->halfEdge() );
		
				splitFace( face, vert1, vert0 );
			}

			// extrude remaining vertices
			for( const auto &vert : extrudedVertices ) {
				if( ! vert.second ) {
					auto normal = glm::any( glm::notEqual( ci::vec3( 0.0f ), extrusion.mDirection ) ) ? glm::normalize( extrusion.mDirection ) : facesNormals[vert.first->halfEdge()->face()];
					vert.first->setPosition( vert.first->getPosition() + normal * offset );
				}
			}

			// see if we need a cap
			if( extrusion.mCreateCap && baseHalfEdge->isBoundary() ) {
				auto end = baseHalfEdge;
				auto boundary = baseHalfEdge->face();
				auto cap = makeFace();
				cap->halfEdge() = baseHalfEdge;
				do {
					baseHalfEdge->face() = cap;
					baseHalfEdge = baseHalfEdge->next();
				} while( baseHalfEdge != end );

				eraseBoundaryFace( boundary );
			}
		}
	}
}
void Mesh::extrude( const std::vector<Face*> &faces, float offset )
{
	extrude( faces, Extrusion().offset( offset ) );
}
void Mesh::extrudeInner( const std::vector<Face*> &faces, float offset )
{
	extrude( faces, Extrusion().innerOffset( offset ) );
}

Face* Mesh::truncate( Vertex* vertex, float radius )
{
	auto it = vertex->halfEdge();

	// find the last HalfEdge
	auto last = it;
	while( last->opposite()->next() != it ) {
		last = last->opposite()->next();
	}
	
	// For each HalfEdge split its vertex while taking away the remaining HalfEdges
	while( it != last ) {
		auto next = it->opposite()->next();
		auto toNext = glm::normalize( it->opposite()->vertex()->getPosition() - it->vertex()->getPosition() );
		auto vert = splitVertex( it->vertex(), it->opposite()->next(), last );
		it->vertex()->setPosition( it->vertex()->getPosition() + toNext * radius );
		it = next;
	}

	// Move the last vertex and add split the last face
	last->vertex()->setPosition( last->vertex()->getPosition() + glm::normalize( last->opposite()->vertex()->getPosition() - last->vertex()->getPosition() ) * radius );
	splitFace( last->opposite()->face(), vertex, last->vertex() );

	return last->opposite()->next()->opposite()->face();
}

void Mesh::transform( const ci::mat4 &mat )
{
	for( const auto &vertex : mVertices ) {
		vertex->setPosition( ci::vec3( mat * ci::vec4( vertex->getPosition(), 1.0f ) ) );
	}
}

// https://www.gamedev.net/resources/_/technical/graphics-programming-and-theory/polygon-triangulation-r3334
void Mesh::triangulate( Face* face )
{
	if( face->size() > 3 ) {
		if( face->isConvex() ) {
			auto a = face->halfEdge()->vertex();
			auto halfEdge = face->halfEdge()->next()->next();
			do {
				splitFace( face, a, halfEdge->vertex() );
				halfEdge = halfEdge->next();
			} while( halfEdge->next() != face->halfEdge() );
		}
	}
}

void Mesh::triangulate()
{
	size_t numFaces = mFaces.size();
	for( size_t i = 0; i < numFaces; ++i ) {
		triangulate( mFaces[i].get() );
	}
}


HalfEdge* Mesh::getHalfEdge( size_t i ) const 
{ 
	return mHalfEdges[i].get(); 
}
Edge* Mesh::getEdge( size_t i ) const 
{ 
	return mEdges[i].get(); 
}
Face* Mesh::getFace( size_t i ) const 
{
	return mFaces[i].get(); 
}
Face* Mesh::getBoundaryFace( size_t i ) const 
{ 
	return mBoundaryFaces[i].get(); 
}
Vertex*	Mesh::getVertex( size_t i ) const 
{ 
	return mVertices[i].get(); 
}

size_t Mesh::getNumHalfEdges() const 
{ 
	return mHalfEdges.size(); 
}
size_t Mesh::getNumEdges() const 
{ 
	return mEdges.size();
}
size_t Mesh::getNumFaces() const 
{ 
	return mFaces.size(); 
}
size_t Mesh::getNumBoundaryFaces() const 
{ 
	return mBoundaryFaces.size(); 
}
size_t Mesh::getNumVertices() const 
{ 
	return mVertices.size(); 
}

const Mesh::HalfEdgeVector&	Mesh::getHalfEdges() const 
{ 
	return mHalfEdges; 
}
const Mesh::EdgeVector&	Mesh::getEdges() const 
{ 
	return mEdges; 
}
const Mesh::FaceVector&	Mesh::getFaces() const 
{ 
	return mFaces; 
}
const Mesh::FaceVector&	Mesh::getBoundaryFaces() const 
{ 
	return mBoundaryFaces; 
}
const Mesh::VertexVector& Mesh::getVertices() const 
{
	return mVertices; 
}

bool Mesh::isTriMesh() const
{
	for( const auto &face : mFaces ) {
		if( face->size() != 3 ) {
			return false;
		}
	}
	return true;
}
bool Mesh::isQuadMesh() const
{
	for( const auto &face : mFaces ) {
		if( face->size() != 4 ) {
			return false;
		}
	}
	return true;
}

ci::gl::VboMeshRef Mesh::createFacesVbo() const
{
	std::vector<ci::vec3> vertices;
	std::vector<ci::vec3> normals;
	
	uint16_t numVertices = mVertices.size();
	std::vector<uint16_t> indices;
	for( const auto &face : mFaces ) {
		
		auto normal = face->calculateNormal();
		auto a = face->halfEdge()->vertex();
		auto b = face->halfEdge()->next()->vertex();
		auto halfEdge = face->halfEdge()->next();
		do {
			auto c = halfEdge->next()->vertex();
			indices.push_back( vertices.size() );
			vertices.push_back( a->getPosition() );
			normals.push_back( normal );
			indices.push_back( vertices.size() );
			vertices.push_back( b->getPosition() );
			normals.push_back( normal );
			indices.push_back( vertices.size() );
			vertices.push_back( c->getPosition() );
			normals.push_back( normal );
			b = c;
			halfEdge = halfEdge->next();
		} while( halfEdge != face->halfEdge() );
	}
		
	auto bufferLayout0 = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::POSITION, 3, 0, 0 ) } );
	auto vbo0 = ci::gl::Vbo::create( GL_ARRAY_BUFFER, vertices );
	auto bufferLayout1 = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::NORMAL, 3, 0, 0 ) } );
	auto vbo1 = ci::gl::Vbo::create( GL_ARRAY_BUFFER, normals );
	auto indexVbo = ci::gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, indices );
	
	return ci::gl::VboMesh::create( vertices.size(), GL_TRIANGLES, { { bufferLayout0, vbo0 }, { bufferLayout1, vbo1 } } );//, indices.size(), GL_UNSIGNED_SHORT, indexVbo );
}


//ci::gl::VboMeshRef Mesh::createFacesVbo() const
//{
//	std::vector<ci::vec3> vertices;
//	std::vector<ci::vec3> normals;
//	std::map<Vertex*,uint16_t> indicesMap;
//	for( const auto &vertex : mVertices ) {
//		indicesMap[vertex.get()] = vertices.size();
//		vertices.push_back( vertex->getPosition() );
//		ci::vec3 normal = ci::vec3( 0.0f );
//		auto it = vertex->halfEdge();
//		do {
//			auto faceNormal = it->face()->calculateNormal();
//			//if( glm::all( glm::equal( normal, ci::vec3( 0.0f ) ) ) || glm::angle( glm::normalize( normal ), faceNormal ) < 0.2f ) {
//				normal += faceNormal;
//			//}
//			it = it->opposite()->next();
//		} while( it != vertex->halfEdge() );
//		normals.push_back( glm::normalize( normal ) );
//	}
//	
//	uint16_t numVertices = mVertices.size();
//	std::vector<uint16_t> indices;
//	for( const auto &edge : mEdges ) {
//		indices.push_back( indicesMap[edge->halfEdge()->vertex()] );
//		indices.push_back( indicesMap[edge->halfEdge()->opposite()->vertex()] );
//	}
//	for( const auto &face : mFaces ) {
//		
//		auto normal = face->calculateNormal();
//		auto a = face->halfEdge()->vertex();
//		auto b = face->halfEdge()->next()->vertex();
//		auto halfEdge = face->halfEdge()->next();
//		do {
//			auto c = halfEdge->next()->vertex();
//			indices.push_back( indicesMap[a] );
//			indices.push_back( indicesMap[b] );
//			indices.push_back( indicesMap[c] );
//			b = c;
//			halfEdge = halfEdge->next();
//		} while( halfEdge != face->halfEdge() );
//	}
//		
//	auto bufferLayout0 = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::POSITION, 3, 0, 0 ) } );
//	auto vbo0 = ci::gl::Vbo::create( GL_ARRAY_BUFFER, vertices );
//	auto bufferLayout1 = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::NORMAL, 3, 0, 0 ) } );
//	auto vbo1 = ci::gl::Vbo::create( GL_ARRAY_BUFFER, normals );
//	auto indexVbo = ci::gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, indices );
//	
//	return ci::gl::VboMesh::create( vertices.size(), GL_TRIANGLES, { { bufferLayout0, vbo0 }, { bufferLayout1, vbo1 } }, indices.size(), GL_UNSIGNED_SHORT, indexVbo );
//}
ci::gl::VboMeshRef Mesh::createEdgesVbo() const
{
	std::map<Vertex*,uint16_t> indicesMap;
	std::vector<ci::vec3> vertices;
	for( const auto &vertex : mVertices ) {
		indicesMap[vertex.get()] = vertices.size();
		vertices.push_back( vertex->getPosition() );
	}

	uint16_t numVertices = mVertices.size();
	std::vector<uint16_t> indices;
	for( const auto &edge : mEdges ) {
		indices.push_back( indicesMap[edge->halfEdge()->vertex()] );
		indices.push_back( indicesMap[edge->halfEdge()->opposite()->vertex()] );
	}

	auto bufferLayout = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::POSITION, 3, 0, 0 ) } );
	auto vbo = ci::gl::Vbo::create( GL_ARRAY_BUFFER, vertices );
	auto indexVbo = ci::gl::Vbo::create( GL_ELEMENT_ARRAY_BUFFER, indices );
	
	return ci::gl::VboMesh::create( vertices.size(), GL_LINES, { { bufferLayout, vbo } }, indices.size(), GL_UNSIGNED_SHORT, indexVbo );
}

ci::gl::VboMeshRef Mesh::createPointsVbo() const
{
	std::vector<ci::vec3> vertices;
	for( const auto &vertex : mVertices ) {
		vertices.push_back( vertex->getPosition() );
	}

	auto bufferLayout = ci::geom::BufferLayout( { ci::geom::AttribInfo( ci::geom::Attrib::POSITION, 3, 0, 0 ) } );
	auto vbo = ci::gl::Vbo::create( GL_ARRAY_BUFFER, vertices );	
	return ci::gl::VboMesh::create( vertices.size(), GL_POINTS, { { bufferLayout, vbo } } );
}

HalfEdge* Mesh::makeHalfEdge()
{
	mHalfEdges.push_back( std::make_unique<HalfEdge>() );
	return mHalfEdges.back().get();
}
Edge* Mesh::makeEdge()
{
	mEdges.push_back( std::make_unique<Edge>() );
	return mEdges.back().get();
}
Face* Mesh::makeFace()
{
	mFaces.push_back( std::make_unique<Face>( false ) );
	return mFaces.back().get();
}
Face* Mesh::makeBoundaryFace()
{
	mBoundaryFaces.push_back( std::make_unique<Face>( true ) );
	return mBoundaryFaces.back().get();
}
Vertex* Mesh::makeVertex( const ci::vec3 &position )
{
	mVertices.push_back( std::make_unique<Vertex>( position ) );
	return mVertices.back().get();
}

void Mesh::eraseHalfEdge( HalfEdge* halfEdge )
{ 
	auto it = std::find_if( mHalfEdges.begin(), mHalfEdges.end(), [halfEdge]( const std::unique_ptr<HalfEdge> &unique ) {
		return unique.get() == halfEdge;
	} );
	if( it != mHalfEdges.end() ) {
		mHalfEdges.erase( it );
	}
	else CI_LOG_E( "Problem erasing HalfEdge" );
}
void Mesh::eraseEdge( Edge* edge )
{
	auto it = std::find_if( mEdges.begin(), mEdges.end(), [edge]( const std::unique_ptr<Edge> &unique ) {
		return unique.get() == edge;
	} );
	if( it != mEdges.end() ) {
		mEdges.erase( it );
	}
	else CI_LOG_E( "Problem erasing Edge" );
}
void Mesh::eraseFace( Face* face )
{
	if( ! face->isBoundary() ) {
		auto it = std::find_if( mFaces.begin(), mFaces.end(), [face]( const std::unique_ptr<Face> &unique ) {
			return unique.get() == face;
		} );
		if( it != mFaces.end() ) {
			mFaces.erase( it );
		}
		else CI_LOG_E( "Problem erasing Face" );
	}
	else {
		eraseBoundaryFace( face );
	}
}
void Mesh::eraseBoundaryFace( Face* face )
{
	if( face->isBoundary() ) {
		auto it = std::find_if( mBoundaryFaces.begin(), mBoundaryFaces.end(), [face]( const std::unique_ptr<Face> &unique ) {
			return unique.get() == face;
		} );
		if( it != mBoundaryFaces.end() ) {
			mBoundaryFaces.erase( it );
		}
		else CI_LOG_E( "Problem erasing Boundary Face" );
	}
	else {
		eraseFace( face );
	}
}
void Mesh::eraseVertex( Vertex* vertex )
{
	auto it = std::find_if( mVertices.begin(), mVertices.end(), [vertex]( const std::unique_ptr<Vertex> &unique ) {
		return unique.get() == vertex;
	} );
	if( it != mVertices.end() ) {
		mVertices.erase( it );
	}
	else CI_LOG_E( "Problem erasing Vertex" );
}


Face::Face( bool boundary ) 
: mIsBoundary( boundary ), mIsSelected( false ), mHalfEdge( nullptr )
{
}

uint16_t Face::size() const
{
	uint16_t n = 0;
	auto h = halfEdge();
	do {
		n++;
		h = h->next();
	} while( h != halfEdge() );
    
	return n;
}

ci::vec3 Face::calculateCentroid() const
{
	ci::vec3 centroid;
	auto h = halfEdge();
	do {
		centroid += h->vertex()->getPosition();
		h = h->next();
	} while( h != halfEdge() );

	return centroid / static_cast<float>( size() );
}
ci::vec3 Face::calculateNormal() const
{
     ci::vec3 n( 0.0f );

     auto halfEdgeIt = halfEdge();
     do {
        n += glm::cross( halfEdgeIt->vertex()->getPosition(), halfEdgeIt->next()->vertex()->getPosition() );
        halfEdgeIt = halfEdgeIt->next();
     }
     while( halfEdgeIt != halfEdge() );

	return glm::normalize( n );
}

ci::AxisAlignedBox Face::calculateBounds() const
{
	ci::vec3 min( std::numeric_limits<float>::max() );
	ci::vec3 max( std::numeric_limits<float>::lowest() );

    auto halfEdgeIt = halfEdge();
    do {
       min = glm::min( min, halfEdgeIt->vertex()->getPosition() );
       max = glm::max( max, halfEdgeIt->vertex()->getPosition() );
       halfEdgeIt = halfEdgeIt->next();
    }
    while( halfEdgeIt != halfEdge() );

	return ci::AxisAlignedBox( min, max );
}

bool Face::intersect( const ci::Ray &r, float * dist ) const
{
	if( isConvex() ) {
		auto a = halfEdge()->vertex();
		auto b = halfEdge()->next()->vertex();
		auto it = halfEdge()->next();
		do {
			auto c = it->next()->vertex();
			if( r.calcTriangleIntersection( a->getPosition(), b->getPosition(), c->getPosition(), dist ) ) {
				return true;
			}

			b = c;
			it = it->next();
		} while( it != halfEdge() );
	}

	return false;
}

bool Face::isSelected() const
{
	return mIsSelected;
}

uint16_t Vertex::degree() const
{
	uint16_t n = 0;
	auto h = halfEdge();
	do {
		if( ! h->face()->isBoundary() ) {
			n++;
		}
		h = h->opposite()->next();
	} while( h != halfEdge() );

	return n;
}


bool Mesh::isValid( bool verbose ) const
{
	bool valid = true;
	for( const auto &halfEdge : mHalfEdges ) {
		if( ! halfEdge->isValid() ) {
			if( verbose ) {
				CI_LOG_D( "halfEdge 0x" << std::hex << halfEdge.get() << " is invalid " );
				valid = false;
			}
			else {
				return false;
			}
		}
	}
	for( const auto &face : mFaces ) {
		if( ! face->isValid() ) {
			if( verbose ) {
				CI_LOG_D( "face 0x" << std::hex << face.get() << " is invalid " );
				valid = false;
			}
			else {
				return false;
			}
		}
	}
	for( const auto &face : mBoundaryFaces ) {
		if( ! face->isValid() ) {
			if( verbose ) {
				CI_LOG_D( " boundary face 0x" << std::hex << face.get() << " is invalid " );
				valid = false;
			}
			else {
				return false;
			}
		}
	}
	for( const auto &edge : mEdges ) {
		if( ! edge->isValid() ) {
			if( verbose ) {
				CI_LOG_D( "edge 0x" << std::hex << edge.get() << " is invalid " );
				valid = false;
			}
			else {
				return false;
			}
		}
	}
	for( const auto &vertex : mVertices ) {
		if( ! vertex->isValid() ) {
			if( verbose ) {
				CI_LOG_D( "vertex 0x" << std::hex << vertex.get() << " is invalid " );
				valid = false;
			}
			else {
				return false;
			}
		}
	}
	return valid;	
}

bool HalfEdge::isBoundary() const
{
	return mFace->isBoundary();
}

bool HalfEdge::isValid() const
{
	return mNext != nullptr 
		&& mOpposite != nullptr 
		&& mFace != nullptr 
		&& mEdge != nullptr 
		&& mVertex != nullptr;
}
bool Face::isValid() const
{
	return mHalfEdge != nullptr && size() > 2;
}
bool Edge::isValid() const
{
	return mHalfEdge != nullptr;
}
bool Vertex::isValid() const
{
	return mHalfEdge != nullptr;
}

} // namespace halfedge

namespace he = halfedge;