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

namespace halfedge {

template<class Output,class Input=Output>
class iterator {
public:
    typedef Output*						value_type;
    typedef Output*&					reference;
    typedef Output*						pointer;
	typedef std::forward_iterator_tag	iterator_category;
	
	iterator();
	iterator( Input* input );
	iterator( const iterator &other );
	virtual ~iterator();
    virtual iterator& operator=( const iterator& it );
    
	virtual iterator& operator++();
	virtual iterator operator++(int);
    virtual value_type operator*() const;
    virtual reference operator*();
    virtual pointer operator->() const;

	explicit operator bool() const;

protected:
	template<class T,class U>
    friend bool operator!=(const iterator<T,U>&, const iterator<T,U>&);
	virtual void advance();

	HalfEdge* mHalfEdge;
	HalfEdge* mStart;
};

//! Constructs and returns a new iterator (same as iterator constructor but with a deductible Input type )
template<class Output, class Input>
iterator<Output,Input> make_iterator( Input* input )
{
	return iterator<Output,Input>( input );
}

// class that enables range-based for loops
template<class Output,class Input>
class iterator_range {
public:
	iterator_range( Input* input ) : mInput( input ) {}
	iterator<Output,Input> begin() const { return iterator<Output,Input>( mInput ); }
	iterator<Output,Input> end() const { return iterator<Output,Input>(); }
protected:
	Input* mInput;
};

//! Constructs and returns a new iterator_range(same as iterator_range constructor but with a deductible Input type )
template<class Output,class Input>
iterator_range<Output,Input> make_range( Input* input ) { return iterator_range<Output,Input>( input ); }

// compare two iterators
template<class Output, class Input>
bool operator!=( const iterator<Output,Input> &lhs, const iterator<Output,Input> &rhs )
{
	return lhs.mHalfEdge != rhs.mHalfEdge || lhs.mStart != lhs.mStart;
}

template<class Output, class Input>
iterator<Output,Input>::iterator() 
: mHalfEdge( nullptr ), mStart( nullptr )
{
}

template<class Output, class Input>
iterator<Output,Input>::iterator( const iterator &other )
: mHalfEdge( other.mHalfEdge ), mStart( other.mStart )
{
}

template<class Output, class Input>
iterator<Output,Input>::iterator( Input* input )
: mHalfEdge( input->halfEdge() ), mStart( input->halfEdge() )
{
}

template<class Output, class Input>
iterator<Output,Input>::~iterator() 
{
}

template<class Output, class Input>
iterator<Output,Input>& iterator<Output,Input>::operator=( const iterator& it )
{
	*this = it;
	return *this;
}

template<class Output, class Input>
iterator<Output,Input>& iterator<Output,Input>::operator++() 
{
	advance();
	return *this;
}

template<class Output, class Input>
iterator<Output,Input> iterator<Output,Input>::operator++(int)
{
	auto previous = iterator( *this );
	advance();
	return previous;
}

template<class Output, class Input>    
void iterator<Output,Input>::advance()
{
	static_assert( sizeof(iterator) != sizeof(iterator), "No he::iterator specialization exists for this combination of Input/Output" );
}

template<class Output, class Input>
iterator<Output,Input>::operator bool() const 
{  
	return mHalfEdge != nullptr;
}

// specializations

// Vertices adjacent to Vertex
template<>    
void iterator<Vertex,Vertex>::advance()
{
	ci::app::console() << "HE: " << std::hex << mHalfEdge << std::dec << std::endl;
	if( mHalfEdge->opposite()->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite()->next();
	}	
}
template<> typename iterator<Vertex,Vertex>::reference iterator<Vertex,Vertex>::operator*() { return mHalfEdge->opposite()->vertex(); }
template<> typename iterator<Vertex,Vertex>::value_type iterator<Vertex,Vertex>::operator*() const { return mHalfEdge->opposite()->vertex(); }
template<> typename iterator<Vertex,Vertex>::pointer iterator<Vertex,Vertex>::operator->() const { return mHalfEdge->opposite()->vertex(); }

// Edges adjacent to Vertex
template<>    
void iterator<Edge,Vertex>::advance()
{
	if( mHalfEdge->opposite()->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite()->next();
	}	
}
template<> typename iterator<Edge,Vertex>::reference iterator<Edge,Vertex>::operator*() { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Vertex>::value_type iterator<Edge,Vertex>::operator*() const { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Vertex>::pointer iterator<Edge,Vertex>::operator->() const { return mHalfEdge->edge(); }

// HalfEdges adjacent to Vertex
template<>    
void iterator<HalfEdge,Vertex>::advance()
{
	if( mHalfEdge->opposite()->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite()->next();
	}	
}
template<> typename iterator<HalfEdge,Vertex>::reference iterator<HalfEdge,Vertex>::operator*() { return mHalfEdge; }
template<> typename iterator<HalfEdge,Vertex>::value_type iterator<HalfEdge,Vertex>::operator*() const { return mHalfEdge; }
template<> typename iterator<HalfEdge,Vertex>::pointer iterator<HalfEdge,Vertex>::operator->() const { return mHalfEdge; }

// Faces adjacent to Vertex
template<>    
void iterator<Face,Vertex>::advance()
{
	if( mHalfEdge->opposite()->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite()->next();
	}	
}
template<> typename iterator<Face,Vertex>::reference iterator<Face,Vertex>::operator*() { return mHalfEdge->face(); }
template<> typename iterator<Face,Vertex>::value_type iterator<Face,Vertex>::operator*() const { return mHalfEdge->face(); }
template<> typename iterator<Face,Vertex>::pointer iterator<Face,Vertex>::operator->() const { return mHalfEdge->face(); }

// Vertices around Face
template<>    
void iterator<Vertex,Face>::advance()
{
	if( mHalfEdge->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->next();
	}	
}
template<> typename iterator<Vertex,Face>::reference iterator<Vertex,Face>::operator*() { return mHalfEdge->vertex(); }
template<> typename iterator<Vertex,Face>::value_type iterator<Vertex,Face>::operator*() const { return mHalfEdge->vertex(); }
template<> typename iterator<Vertex,Face>::pointer iterator<Vertex,Face>::operator->() const { return mHalfEdge->vertex(); }

// Edges around Face
template<>    
void iterator<Edge,Face>::advance()
{
	if( mHalfEdge->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->next();
	}	
}
template<> typename iterator<Edge,Face>::reference iterator<Edge,Face>::operator*() { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Face>::value_type iterator<Edge,Face>::operator*() const { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Face>::pointer iterator<Edge,Face>::operator->() const { return mHalfEdge->edge(); }

// HalfEdges around Face
template<>    
void iterator<HalfEdge,Face>::advance()
{
	if( mHalfEdge->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->next();
	}	
}
template<> typename iterator<HalfEdge,Face>::reference iterator<HalfEdge,Face>::operator*() { return mHalfEdge; }
template<> typename iterator<HalfEdge,Face>::value_type iterator<HalfEdge,Face>::operator*() const { return mHalfEdge; }
template<> typename iterator<HalfEdge,Face>::pointer iterator<HalfEdge,Face>::operator->() const { return mHalfEdge; }

// Faces adjacent to Face
template<>    
void iterator<Face,Face>::advance()
{
	auto face = mHalfEdge->face();
	while( mHalfEdge && mHalfEdge->face() == face ) {
		if( mHalfEdge->next() == mStart ) {
			mHalfEdge = nullptr;
		}
		else {
			mHalfEdge = mHalfEdge->next();
		}
	}
}
template<> typename iterator<Face,Face>::reference iterator<Face,Face>::operator*() { return mHalfEdge->face(); }
template<> typename iterator<Face,Face>::value_type iterator<Face,Face>::operator*() const { return mHalfEdge->face(); }
template<> typename iterator<Face,Face>::pointer iterator<Face,Face>::operator->() const { return mHalfEdge->face(); }

// EV EE EF
template<>    
void iterator<Vertex,Edge>::advance()
{
	if( mHalfEdge->opposite() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite();
	}	
}
template<> typename iterator<Vertex,Edge>::reference iterator<Vertex,Edge>::operator*() { return mHalfEdge->vertex(); }
template<> typename iterator<Vertex,Edge>::value_type iterator<Vertex,Edge>::operator*() const { return mHalfEdge->vertex(); }
template<> typename iterator<Vertex,Edge>::pointer iterator<Vertex,Edge>::operator->() const { return mHalfEdge->vertex(); }

template<>    
void iterator<Edge,Edge>::advance()
{
	if( mHalfEdge->opposite() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite();
	}	
}
template<> typename iterator<Edge,Edge>::reference iterator<Edge,Edge>::operator*() { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Edge>::value_type iterator<Edge,Edge>::operator*() const { return mHalfEdge->edge(); }
template<> typename iterator<Edge,Edge>::pointer iterator<Edge,Edge>::operator->() const { return mHalfEdge->edge(); }

template<>    
void iterator<Face,Edge>::advance()
{
	if( mHalfEdge->opposite() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->opposite();
	}	
}
template<> typename iterator<Face,Edge>::reference iterator<Face,Edge>::operator*() { return mHalfEdge->face(); }
template<> typename iterator<Face,Edge>::value_type iterator<Face,Edge>::operator*() const { return mHalfEdge->face(); }
template<> typename iterator<Face,Edge>::pointer iterator<Face,Edge>::operator->() const { return mHalfEdge->face(); }

// HalfEdges
template<>
iterator<HalfEdge,HalfEdge>::iterator( HalfEdge* halfEdge )
: mHalfEdge( halfEdge ), mStart( halfEdge )
{
}

template<>    
void iterator<HalfEdge,HalfEdge>::advance()
{
	if( mHalfEdge->next() == mStart ) {
		mHalfEdge = nullptr;
	}
	else {
		mHalfEdge = mHalfEdge->next();
	}	
}
template<> typename iterator<HalfEdge,HalfEdge>::reference iterator<HalfEdge,HalfEdge>::operator*() { return mHalfEdge; }
template<> typename iterator<HalfEdge,HalfEdge>::value_type iterator<HalfEdge,HalfEdge>::operator*() const { return mHalfEdge; }
template<> typename iterator<HalfEdge,HalfEdge>::pointer iterator<HalfEdge,HalfEdge>::operator->() const { return mHalfEdge; }

} // namespace halfedge