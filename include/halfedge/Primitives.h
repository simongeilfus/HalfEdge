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

#include "halfedge/Mesh.h"
#include "cinder/vector.h"

namespace halfedge { namespace geom {

class Polygon : public MeshSource {
public:
	// Specifies the number of 
	virtual Polygon& subdivisions( const uint16_t &subdivisions ) { mSubdivisions = glm::max( subdivisions, (uint16_t) 3 ); return *this; }
	//! Specifies the 
	virtual Polygon& radius( float radius ) { mRadius = radius; return *this; }
	//! Specifies the size in each axis. Defaults to [2, 2], or 1 in each direction
	virtual Polygon& center( const ci::vec3 &center ) { mCenter = center; return *this; }
	//!
	//virtual Polygon& axes( const ci::vec3 &uAxis, const ci::vec3 &vAxis );
	Polygon() : mSubdivisions( 3 ), mRadius( 1.0f ), mCenter( 0.0f ) {}
	virtual void loadInto( Mesh* target ) const override;
protected:
	uint16_t	mSubdivisions;
	float		mRadius;
	ci::vec3	mCenter;
};

class Plane : public MeshSource {
public:
	Plane() : mSubdivisions( 1 ), mSize( 1.0f ), mCenter( 0.0f ) {}

	// Specifies the number of 
	virtual Plane& subdivisions( const ci::ivec2 &subdivisions ) { mSubdivisions = glm::max( subdivisions, ci::ivec2(1) ); return *this; }
	//! Specifies the 
	virtual Plane& size( const ci::vec2 &size ) { mSize = size; return *this; }
	//! Specifies the size in each axis. Defaults to [2, 2], or 1 in each direction
	virtual Plane& center( const ci::vec3 &center ) { mCenter = center; return *this; }
	//!
	//virtual Plane& axes( const ci::vec3 &uAxis, const ci::vec3 &vAxis );
	virtual void loadInto( Mesh* target ) const override;
protected:
	ci::ivec2	mSubdivisions;
	ci::vec2	mSize;
	ci::vec3	mCenter;
};

class Cube : public Plane {
public:
	// Specifies the number of 
	virtual Cube& subdivisions( const ci::ivec3 &subdivisions ) { mSubdivisions = glm::max( subdivisions, ci::ivec3(1) ); updatePlane(); return *this; }
	//! Specifies the 
	virtual Cube& size( const ci::vec3 &size ) { mSize = size; updatePlane(); return *this; }
	//! Specifies the size in each axis. Defaults to [2, 2], or 1 in each direction
	virtual Cube& center( const ci::vec3 &center ) { mCenter = center; return *this; }

	virtual void loadInto( Mesh* target ) const override;

	Cube() : mSubdivisions( 1 ), mSize( 1.0f ) {}
protected:
	void updatePlane();
	ci::ivec3 mSubdivisions;
	ci::vec3  mSize;
};

class Cylinder : public Polygon {
public:

	//! Specifices the base of the Cylinder.
	Cylinder&	center( const ci::vec3 &center ) { mCenter = center; return *this; }
	//! Specifies the height of the cylinder.
	Cylinder&	height( float height ) { mHeight = height; return *this; }
	//! Specifies the base and apex radius.
	Cylinder&	radius( float radius ) { mRadius = glm::max(0.f, radius); return *this; }
	//! Specifies the axis of the cylinder.
	Cylinder&	direction( const ci::vec3 &direction ) { mDirection = normalize( direction ); return *this; }
	//! Specifies the number of radial subdivisions, which determines the roundness of the Cylinder. Defaults to \c 18.
	Cylinder&	subdivisions( int subdiv ) { mSubdivisions = subdiv; return *this; }
	//! Specifies the number of slices along the Cylinder's height. Defaults to \c 1.
	Cylinder&	subdivisionsHeight( int slices ) { mSubdivisionsHeight = slices; return *this; }
	//! Specifies the number of rings for the Cylinder's cap. Defaults to \c 3.
	Cylinder&	subdivisionsCap( int rings ) { mSubdivisionsCap = rings; return *this; }
	//! Specifies the height of the cylinder.
	Cylinder&	cap( bool cap = true ) { mCap = cap; return *this; }
	//! Conveniently sets origin, height and direction so that the center of the base is \a from and the center of the apex is \a to.
	Cylinder&	set( const ci::vec3 &from, const ci::vec3 &to );
	
	Cylinder::Cylinder() : mHeight( 2.0f ), mDirection( 0, 1, 0 ), mSubdivisionsHeight( 1 ), mCap( true ), mSubdivisionsCap( 3 ), 
	Polygon( Polygon().radius( 1.0f ).subdivisions( 18 ).center( ci::vec3( 0.0f, 0.0f, 0.0f ) ) ) {}

	virtual void loadInto( Mesh* target ) const override;
protected:
	bool		mCap;
	float		mHeight;
	ci::vec3	mDirection;
	int			mSubdivisionsHeight;
	int			mSubdivisionsCap;
	bool		mHasColors;
	int	mNumSegments, mNumSlices;
};

class Cone : public Polygon {
public:
	virtual void loadInto( Mesh* target ) const override;
};

class Sphere : public MeshSource {
public:
	virtual void loadInto( Mesh* target ) const override;
};

class Ring : public MeshSource {
public:
	// Specifies the number of 
	virtual Ring& subdivisions( const uint16_t &subdivisions ) { mSubdivisions = glm::max( subdivisions, (uint16_t) 3 ); return *this; }
	//! Specifies the 
	virtual Ring& outerRadius( float radius ) { mOuterRadius = radius; return *this; }
	virtual Ring& innerRadius( float radius ) { mInnerRadius = radius; return *this; }
	//! Specifies the size in each axis. Defaults to [2, 2], or 1 in each direction
	virtual Ring& center( const ci::vec3 &center ) { mCenter = center; return *this; }
	
	Ring() : mSubdivisions( 18 ), mOuterRadius( 1.0f ), mInnerRadius( 0.75f ), mCenter( 0.0f ) {}
	virtual void loadInto( Mesh* target ) const override;
protected:
	uint16_t	mSubdivisions;
	float		mOuterRadius;
	float		mInnerRadius;
	ci::vec3	mCenter;
};

class Tube : public Ring {
public:
	// Specifies the number of 
	Ring& subdivisions( const uint16_t &subdivisions ) { mSubdivisions = glm::max( subdivisions, (uint16_t) 1 ); return *this; }
	Ring& subdivisionsHeight( const uint16_t &subdivisions ) { mSubdivisionsHeight = glm::max( subdivisions, (uint16_t) 1 ); return *this; }
	//! Specifies the 
	Ring& outerRadius( float radius ) { mOuterRadius = radius; return *this; }
	Ring& innerRadius( float radius ) { mInnerRadius = radius; return *this; }
	//! Specifies the height of the cylinder.
	Ring& height( float height ) { mHeight = height; return *this; }
	//! Specifies the size in each axis. Defaults to [2, 2], or 1 in each direction
	Ring& center( const ci::vec3 &center ) { mCenter = center; return *this; }
	virtual void loadInto( Mesh* target ) const override;
	
	Tube() : mSubdivisionsHeight( 1 ), mHeight( 1.0f ) {}
protected:
	float mHeight;
	uint16_t	mSubdivisionsHeight;
};

}} // namespace halfhedge::geom
