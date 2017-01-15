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

#include "halfedge/Primitives.h"

namespace halfedge { namespace geom {

void Polygon::loadInto( Mesh* target ) const
{
	std::vector<ci::vec3> vertices;
	std::vector<uint32_t> indices;
	float invAngle = 1.0f / (float) mSubdivisions * 2.0f * M_PI;// - M_PI * 0.25f;
	for( size_t i = 0; i < mSubdivisions; ++i ) {
		float angle = (float) i * invAngle;
		vertices.push_back( mCenter + ci::vec3( glm::sin( angle ), 0.0f, glm::cos( angle ) ) * mRadius );
		indices.push_back( i );
	}

	addFaces( target, vertices, { indices } );
}

void Plane::loadInto( Mesh* target ) const
{
	auto halfSize = mSize * 0.5f;
	addFaces( target, {
		mCenter + ci::vec3( -halfSize.x, 0, -halfSize.y ),
		mCenter + ci::vec3( -halfSize.x, 0, halfSize.y ),
		mCenter + ci::vec3( halfSize.x, 0, halfSize.y ),
		mCenter + ci::vec3( halfSize.x, 0, -halfSize.y )
	}, {
		{ 0, 1, 2, 3 },
	} );

	auto face = target->getFaces()[0].get();
	auto he0 = target->getHalfEdges()[0].get();
	auto he1 = he0->next()->next();
		
	auto he0Start = he0->vertex()->getPosition();
	auto he0End = he0->next()->vertex()->getPosition();
	auto he1Start = he1->vertex()->getPosition();
	auto he1End = he1->next()->vertex()->getPosition();
	for( size_t i = 1; i < mSubdivisions.x; ++i ) {
		auto split0 = mix( he0Start, he0End, (float) ( i ) / (float) ( mSubdivisions.x ) );
		auto split1 = mix( he1End, he1Start, (float) ( i ) / (float) ( mSubdivisions.x ) );
		auto vert0 = target->splitEdge( he0->edge(), split0 );
		he0 = vert0->halfEdge();
		auto vert1 = target->splitEdge( he1->edge(), split1 );
		target->splitFace( vert0->halfEdge()->face(), vert0, vert1 );
	}

		
	he0 = target->getHalfEdges()[0].get()->next()->next()->next();
		
	he0Start = he0->vertex()->getPosition();
	he0End = he0->next()->vertex()->getPosition();
	for( size_t i = 1; i < mSubdivisions.y; ++i ) {
		auto split0 = mix( he0Start, he0End, (float) ( i ) / (float) ( mSubdivisions.y ) );
		auto vert0 = target->splitEdge( he0->edge(), split0 );
		he0 = vert0->halfEdge();
		he1 = he0->next()->next();
		auto oppositeHe0 = he0->opposite();
		for( size_t j = 0; j < i + 2; j++ ) {
			oppositeHe0 = oppositeHe0->next();
		}
		for( size_t j = 1; j <= mSubdivisions.x; ++j ) {
			he1Start = he1->vertex()->getPosition();
			he1End = oppositeHe0->vertex()->getPosition();
			auto split1 = mix( he1End, he1Start, (float) ( i ) / (float) ( mSubdivisions.y ) );
			auto vert1 = target->splitEdge( he1->edge(), split1 );
			target->splitFace( he1->face(), he1->next()->vertex(), vert0 );
			he1 = he1->opposite()->next()->next();
			vert0 = vert1;
			oppositeHe0 = oppositeHe0->next();
		}
	}
}

void Cube::loadInto( Mesh* target ) const
{
	Plane::loadInto( target );

	// extrude vertically
	target->extrude( target->getFace( 0 ), Mesh::Extrusion().steps( mSubdivisions.y ).offset( mSize.y ).cap() );
	
	std::vector<Face*> faces;
	// extrude all the faces on the Z+ side
	for( size_t i = 0, n = target->getNumFaces(); i < n; ++i ) {
		if( glm::distance( ci::vec3( 0, 0, 1 ), target->getFace( i )->calculateNormal() ) < 0.001f ) {
			faces.push_back( target->getFace( i ) );
		}
	}
	target->extrude( faces, Mesh::Extrusion().steps( mSubdivisions.z - 1 ).offset( mSize.z - mSize.z / (float) mSubdivisions.z ) );
	
	// extrude all the faces on the X+ side
	faces.clear();
	for( size_t i = 0, n = target->getNumFaces(); i < n; ++i ) {
		if( glm::distance( ci::vec3( 1, 0, 0 ), target->getFace( i )->calculateNormal() ) < 0.001f ) {
			faces.push_back( target->getFace( i ) );
		}
	}
	target->extrude( faces, Mesh::Extrusion().steps( mSubdivisions.x - 1 ).offset( mSize.x - mSize.x / (float) mSubdivisions.x ) );
}

void Cube::updatePlane()
{
	 Plane::size( ci::vec2( mSize.x, mSize.z ) / ci::vec2( mSubdivisions.x, mSubdivisions.z ) );
	 Plane::center( ci::vec3( -mSize.x * 0.5f + mSize.x / (float) mSubdivisions.x * 0.5f, 
							  0.0f, 
							  -mSize.z * 0.5f + mSize.z / (float) mSubdivisions.z * 0.5f ) );
}

Cylinder& Cylinder::set( const ci::vec3 &from, const ci::vec3 &to )
{
	const ci::vec3 axis = ( to - from );
	mHeight = glm::length( axis );
	mDirection = glm::normalize( axis );
	mCenter = from;
	return *this;
}

void Cylinder::loadInto( Mesh* target ) const
{
	Polygon::loadInto( target );
	auto face = target->getFace( 0 );
	target->extrude( face, he::Mesh::Extrusion().offset( mHeight ).steps( mSubdivisionsHeight ).cap( mCap ) );
	
	
	if( mSubdivisionsCap > 1 ) {
		if( mCap ) {
			auto cap = target->getFace( target->getNumFaces() - 1 );
			target->extrude( cap, Mesh::Extrusion().innerOffset( 2.0f * mRadius / (float) ( mSubdivisionsCap ) ).steps( mSubdivisionsCap - 1 ) );
			target->splitFace( cap );
		}
		target->extrude( face, Mesh::Extrusion().innerOffset( 2.0f * mRadius / (float) ( mSubdivisionsCap ) ).steps( mSubdivisionsCap - 1 ) );
		target->splitFace( face );
	}
}

void Cone::loadInto( Mesh* target ) const
{
}
void Sphere::loadInto( Mesh* target ) const
{
}

void Ring::loadInto( Mesh* target ) const
{
	std::vector<glm::vec3> vertices;
	std::vector<std::vector<uint32_t>> indices;
	for( uint32_t i = 0; i < mSubdivisions; ++i ) {
		float angle = (float) i / (float) mSubdivisions * 2.0f * M_PI;
		vertices.push_back( mCenter + glm::vec3( glm::sin( angle ), 0.0f, glm::cos( angle ) ) * mInnerRadius );
		vertices.push_back( mCenter + glm::vec3( glm::sin( angle ), 0.0f, glm::cos( angle ) ) * mOuterRadius );
		indices.push_back( { 
			( i + 1 ) % mSubdivisions * 2, 
			i * 2, 
			i * 2 + 1, 
			( i + 1 ) % mSubdivisions * 2 + 1
		} );
	}
	
	addFaces( target, vertices, indices );
}

void Tube::loadInto( Mesh* target ) const
{
	Ring::loadInto( target );
	std::vector<Face*> faces;
	for( size_t i = 0; i < target->getNumFaces(); ++i ) {
		faces.push_back( target->getFace( i ) );
	}

	target->extrude( faces, Mesh::Extrusion().offset( mHeight ).cap(false).steps( mSubdivisionsHeight ) );
}

} } // namespace halfedge::geom

