// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "math_internal.h"
#include "utils.h"
#include "test_macros.h"

#include <float.h>
#include <stdio.h>

// 0.0023 degrees
#define ATAN_TOL 0.00004f

int MathTest( void )
{
	for ( float t = -10.0f; t < 10.0f; t += 0.01f )
	{
		float angle = B3_PI * t;
		b3CosSin cs = b3ComputeCosSin( angle );
		float c = cosf( angle );
		float s = sinf( angle );

		// The cosine and sine approximations are accurate to about 0.1 degrees (0.002 radians)
		// printf( "%g %g\n", r.c - c, r.s - s );
		ENSURE_SMALL( cs.cosine - c, 0.002f );
		ENSURE_SMALL( cs.sine - s, 0.002f );

		float xn = b3UnwindAngle( angle );
		float a = b3Atan2( s, c );
		ENSURE( b3IsValidFloat( a ) );

		float diff = b3AbsFloat( a - xn );

		// The two results can be off by 360 degrees (-pi and pi)
		if ( diff > B3_PI )
		{
			diff -= 2.0f * B3_PI;
		}

		// The approximate atan2 is quite accurate
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	for ( float y = -1.0f; y <= 1.0f; y += 0.01f )
	{
		for ( float x = -1.0f; x <= 1.0f; x += 0.01f )
		{
			float a1 = b3Atan2( y, x );
			float a2 = atan2f( y, x );
			float diff = b3AbsFloat( a1 - a2 );
			ENSURE( b3IsValidFloat( a1 ) );
			ENSURE_SMALL( diff, ATAN_TOL );
		}
	}

	{
		float a1 = b3Atan2( 1.0f, 0.0f );
		float a2 = atan2f( 1.0f, 0.0f );
		float diff = b3AbsFloat( a1 - a2 );
		ENSURE( b3IsValidFloat( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b3Atan2( -1.0f, 0.0f );
		float a2 = atan2f( -1.0f, 0.0f );
		float diff = b3AbsFloat( a1 - a2 );
		ENSURE( b3IsValidFloat( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b3Atan2( 0.0f, 1.0f );
		float a2 = atan2f( 0.0f, 1.0f );
		float diff = b3AbsFloat( a1 - a2 );
		ENSURE( b3IsValidFloat( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b3Atan2( 0.0f, -1.0f );
		float a2 = atan2f( 0.0f, -1.0f );
		float diff = b3AbsFloat( a1 - a2 );
		ENSURE( b3IsValidFloat( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	{
		float a1 = b3Atan2( 0.0f, 0.0f );
		float a2 = atan2f( 0.0f, 0.0f );
		float diff = b3AbsFloat( a1 - a2 );
		ENSURE( b3IsValidFloat( a1 ) );
		ENSURE_SMALL( diff, ATAN_TOL );
	}

	b3Vec3 zero = { 0 };
	b3Vec3 one = { 1.0f, 1.0f, 1.0f };
	b3Vec3 two = { 2.0f, 2.0f, 2.0f };

	b3Vec3 v = b3Add( one, two );
	ENSURE( v.x == 3.0f && v.y == 3.0f );

	v = b3Sub( zero, two );
	ENSURE( v.x == -2.0f && v.y == -2.0f );

	v = b3Add( two, two );
	ENSURE( v.x != 5.0f && v.y != 5.0f );

	b3Vec3 axis = b3Normalize( (b3Vec3){ -0.75f, 0.5f, 1.0f } );
	b3Transform transform1 = { { -2.0f, 3.0f, 0.0f }, b3Quat_identity };
	b3Transform transform2 = { { 1.0f, 0.0f, 0.0f }, b3MakeQuatFromAxisAngle( axis, B3_PI ) };

	b3Transform transform = b3MulTransforms( transform2, transform1 );

	v = b3TransformPoint( transform2, b3TransformPoint( transform1, two ) );

	b3Vec3 u = b3TransformPoint( transform, two );

	ENSURE_SMALL( u.x - v.x, 10.0f * FLT_EPSILON );
	ENSURE_SMALL( u.y - v.y, 10.0f * FLT_EPSILON );

	v = b3TransformPoint( transform1, two );
	v = b3InvTransformPoint( transform1, v );

	ENSURE_SMALL( v.x - two.x, 8.0f * FLT_EPSILON );
	ENSURE_SMALL( v.y - two.y, 8.0f * FLT_EPSILON );

	b3Transform relTransform = b3InvMulTransforms( transform1, transform2 );
	v = b3InvTransformPoint( transform1, b3TransformPoint( transform2, two ) );
	u = b3TransformPoint( relTransform, two );
	ENSURE_SMALL( u.x - v.x, 10.0f * FLT_EPSILON );
	ENSURE_SMALL( u.y - v.y, 10.0f * FLT_EPSILON );

	{
		axis = (b3Vec3){ 0.0f, 0.0f, 1.0f };
		b3Quat q1 = b3MakeQuatFromAxisAngle( axis, -0.5f * B3_PI );
		b3Quat q2 = b3ComputeQuatBetweenUnitVectors( (b3Vec3){ 1.0f, 0.0f, 0.0f }, (b3Vec3){ 0.0f, -1.0f, 0.0f } );

		ENSURE_SMALL( q1.v.x - q2.v.x, FLT_EPSILON );
		ENSURE_SMALL( q1.v.y - q2.v.y, FLT_EPSILON );
		ENSURE_SMALL( q1.v.z - q2.v.z, FLT_EPSILON );
		ENSURE_SMALL( q1.s - q2.s, FLT_EPSILON );

		b3Quat q3 = b3NormalizeQuat( (b3Quat){ { 1.0f, -2.0f, 3.0f }, 4.0f } );
		b3Quat q4 = b3InvMulQuat( q3, q1 );
		b3Quat q5 = b3MulQuat( q3, q4 );
		ENSURE_SMALL( q1.v.x - q5.v.x, FLT_EPSILON );
		ENSURE_SMALL( q1.v.y - q5.v.y, FLT_EPSILON );
		ENSURE_SMALL( q1.v.z - q5.v.z, FLT_EPSILON );
		ENSURE_SMALL( q1.s - q5.s, FLT_EPSILON );

		b3Quat q6 = b3ComputeQuatBetweenUnitVectors( (b3Vec3){ 0.0f, 1.0f, 0.0f }, (b3Vec3){ 0.0f, -1.0f, 0.0f } );
		ENSURE_SMALL( q6.s, FLT_EPSILON );
		(void)q6;
	}

	v = b3Normalize( (b3Vec3){ 0.2f, -0.5f, 3.0f } );
	for ( float z = -1.0f; z <= 1.0f; z += 0.02f )
	{
		for ( float y = -1.0f; y <= 1.0f; y += 0.02f )
		{
			for ( float x = -1.0f; x <= 1.0f; x += 0.02f )
			{
				if ( x == 0.0f && y == 0.0f && z == 0.0f )
				{
					continue;
				}

				u = b3Normalize( (b3Vec3){ x, y, z } );

				b3Quat r = b3ComputeQuatBetweenUnitVectors( v, u );
				ENSURE( b3IsValidQuat( r ) );

				b3Vec3 w = b3RotateVector( r, v );

				ENSURE_SMALL( b3Dot( r.v, b3Cross( u, w ) ) - b3ScalarTripleProduct( r.v, u, w ), FLT_EPSILON );

				// The quaternion between vectors can have lots of round off error at large angles.
				ENSURE_SMALL( w.x - u.x, 0.001f );
				ENSURE_SMALL( w.y - u.y, 0.001f );
				ENSURE_SMALL( w.z - u.z, 0.001f );

				// Twist angle testing
				float twist = r.s < 0.0f ? b3Atan2( -r.v.z, -r.s ) : b3Atan2( r.v.z, r.s );
				twist *= 2.0f;
				ENSURE( -B3_PI <= twist && twist <= B3_PI );
			}
		}
	}

	{
		// More twist angle testing
		b3Quat q = { .v = { -0.0558656752f, -0.188799798f, 0.00689807534f }, .s = -0.980401039f };
		float twist = q.s < 0.0f ? b3Atan2( -q.v.z, -q.s ) : b3Atan2( q.v.z, q.s );
		twist *= 2.0f;
		ENSURE( -B3_PI <= twist && twist <= B3_PI );
	}

	{
		b3Matrix3 m = { { 3.0f, 1.0f, -1.0f }, { -1.0f, 3.0f, 1.0f }, { 1.0f, -1.0f, 3.0f } };
		b3Matrix3 invM = b3InvertMatrix( m );
		b3Matrix3 a = b3MulMM( m, invM );
		ENSURE_SMALL( a.cx.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( a.cx.y, FLT_EPSILON );
		ENSURE_SMALL( a.cx.z, FLT_EPSILON );
		ENSURE_SMALL( a.cy.x, FLT_EPSILON );
		ENSURE_SMALL( a.cy.y - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( a.cy.z, FLT_EPSILON );
		ENSURE_SMALL( a.cz.x, FLT_EPSILON );
		ENSURE_SMALL( a.cz.y, FLT_EPSILON );
		ENSURE_SMALL( a.cz.z - 1.0f, FLT_EPSILON );

		v = (b3Vec3){ 1.0f, -2.0f, 3.0f };
		u = b3MulMV( invM, b3MulMV( m, v ) );
		ENSURE_SMALL( v.x - u.x, FLT_EPSILON );
		ENSURE_SMALL( v.y - u.y, FLT_EPSILON );
		ENSURE_SMALL( v.z - u.z, FLT_EPSILON );

		b3Vec3 w = b3MulMV( invM, v );
		u = b3Solve3( m, v );
		ENSURE_SMALL( w.x - u.x, FLT_EPSILON );
		ENSURE_SMALL( w.y - u.y, FLT_EPSILON );
		ENSURE_SMALL( w.z - u.z, FLT_EPSILON );
	}

	{
		b3Matrix2 m = { { 3.0f, 1.0f }, { -1.0f, 3.0f } };
		b3Matrix2 invM = b3Invert2( m );
		b3Matrix2 a = b3MulMM2( m, invM );
		ENSURE_SMALL( a.cx.x - 1.0f, FLT_EPSILON );
		ENSURE_SMALL( a.cx.y, FLT_EPSILON );
		ENSURE_SMALL( a.cy.x, FLT_EPSILON );
		ENSURE_SMALL( a.cy.y - 1.0f, FLT_EPSILON );

		b3Vec2 v2 = { 1.0f, -2.0f };
		b3Vec2 u2 = b3MulMV2( invM, b3MulMV2( m, v2 ) );
		ENSURE_SMALL( v2.x - u2.x, FLT_EPSILON );
		ENSURE_SMALL( v2.y - u2.y, FLT_EPSILON );

		b3Vec2 w = b3MulMV2( invM, v2 );
		u2 = b3Solve2( m, v2 );
		ENSURE_SMALL( w.x - u2.x, FLT_EPSILON );
		ENSURE_SMALL( w.y - u2.y, FLT_EPSILON );

		w = b3MulMV2( m, u2 );
		ENSURE_SMALL( w.x - v2.x, 10.0f * FLT_EPSILON );
		ENSURE_SMALL( w.y - v2.y, 10.0f * FLT_EPSILON );
	}

	for (int i = 0; i < 100; ++i)
	{
		float a = RandomFloat();
		double b = a;
		float c = (float)b;
		ENSURE( c == a );
	}

	b3Quat q1 = b3Quat_identity;
	b3Quat q2 = b3MakeQuatFromAxisAngle(b3Vec3_axisZ, 0.5f * B3_PI );
	int n = 100;
	for ( int i = 0; i <= n; ++i )
	{
		float alpha = (float)i / (float)n;
		b3Quat q = b3NLerp( q1, q2, alpha );
		float angle = b3GetTwistAngle( q );
		ENSURE_SMALL( alpha * 0.5f * B3_PI - angle, 1.0f * B3_DEG_TO_RAD );
		//printf( "angle = [%g %g %g]\n", alpha, alpha * 0.5f * B3_PI, angle );
	}

	{
		b3Vec3 normal = { 0.504055440f, 0.621548057f, 0.599671543f };
		b3Vec3 perp = b3ArbitraryPerp( normal );
		ENSURE_SMALL( b3Dot( normal, perp ), 2.0f * FLT_EPSILON );
	}

	return 0;
}
