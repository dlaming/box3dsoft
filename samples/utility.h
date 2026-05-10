// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box3d/types.h"

struct Vector4
{
	float x, y, z, w;
};

struct Matrix4
{
	Vector4 cx, cy, cz, cw;
};

inline Matrix4 MakeMatrix4( const b3Transform& transform, b3Vec3 scale )
{
	b3Matrix3 m3 = b3MakeMatrixFromQuat( transform.q );
	Matrix4 m = {};
	m.cx = { scale.x * m3.cx.x, scale.x * m3.cx.y, scale.x * m3.cx.z, 0.0f };
	m.cy = { scale.y * m3.cy.x, scale.y * m3.cy.y, scale.y * m3.cy.z, 0.0f };
	m.cz = { scale.z * m3.cz.x, scale.z * m3.cz.y, scale.z * m3.cz.z, 0.0f };
	m.cw = { transform.p.x, transform.p.y, transform.p.z, 1.0f };
	return m;
}

inline Matrix4 MakeMatrix4( const b3Transform& transform, float scale )
{
	b3Matrix3 m3 = b3MakeMatrixFromQuat( transform.q );
	Matrix4 m = {};
	m.cx = { scale * m3.cx.x, scale * m3.cx.y, scale * m3.cx.z, 0.0f };
	m.cy = { scale * m3.cy.x, scale * m3.cy.y, scale * m3.cy.z, 0.0f };
	m.cz = { scale * m3.cz.x, scale * m3.cz.y, scale * m3.cz.z, 0.0f };
	m.cw = { transform.p.x, transform.p.y, transform.p.z, 1.0f };
	return m;
}

inline Vector4 MulMV( const Matrix4& m, Vector4 a )
{
	Vector4 b;
	b.x = m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z + m.cw.x * a.w;
	b.y = m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z + m.cw.y * a.w;
	b.z = m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z + m.cw.z * a.w;
	b.w = m.cx.w * a.x + m.cy.w * a.y + m.cz.w * a.z + m.cw.w * a.w;
	return b;
}

inline Matrix4 MulMM( const Matrix4& a, const Matrix4& b )
{
	Matrix4 c;
	c.cx = MulMV( a, b.cx );
	c.cy = MulMV( a, b.cy );
	c.cz = MulMV( a, b.cz );
	c.cw = MulMV( a, b.cw );
	return c;
}

struct RGBA8
{
	uint8_t r, g, b, a;
};

inline RGBA8 MakeRGBA8( b3HexColor h, float alpha )
{
	RGBA8 color;
	color.r = ( h >> 16 ) & 0xFF;
	color.g = ( h >> 8 ) & 0xFF;
	color.b = h & 0xFF;
	color.a = uint8_t( 255.0f * alpha + 0.5f );
	return color;
}

inline RGBA8 MakeRGBA8( b3HexColor h )
{
	RGBA8 color;
	color.r = ( h >> 16 ) & 0xFF;
	color.g = ( h >> 8 ) & 0xFF;
	color.b = h & 0xFF;
	color.a = 0xFF;
	return color;
}

struct RGBAf
{
	float r, g, b, a;
};

inline RGBAf MakeRGBAf( b3HexColor h, float alpha )
{
	RGBAf color;
	color.r = ( ( h >> 16 ) & 0xFF ) / 255.0f;
	color.g = ( ( h >> 8 ) & 0xFF ) / 255.0f;
	color.b = ( h & 0xFF ) / 255.0f;
	color.a = alpha;
	return color;
}

inline RGBAf MakeRGBAf( b3HexColor h )
{
	return MakeRGBAf( h, 1.0f );
}

Matrix4 MakeOrthographicMatrix( float left, float right, float bottom, float top, float near, float far );

struct Arena
{
	void Create( int byteCount );
	void Destroy();

	void* Allocate( int byteCount );
	void Clear();

	char* data;
	int capacity;
	int index;
};
