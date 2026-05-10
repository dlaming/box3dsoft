// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/collision.h"

static int ParallelPlanes( void )
{
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[0].plane.offset = 0.5f;
	planes[0].pushLimit = FLT_MAX;
	planes[1].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[1].plane.offset = 1.0f;
	planes[1].pushLimit = FLT_MAX;
	// planes[2].plane.normal = b3Normalize((b3Vec3){ 0.2f, 0.0f, 0.9f });
	// planes[2].plane.offset = 0.25f;
	// planes[2].pushLimit = FLT_MAX;

	b3Vec3 target = { 0.0f, 0.0f, 0.0f };
	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 2 );
	ENSURE_SMALL( result.delta.z - 1.0f, 0.0055f );

	return 0;
}

static int GamePlanes( void )
{
	// This scenario takes many iterations because the target is deep into the plane.
	b3CollisionPlane planes[3] = { 0 };
	planes[0].plane.normal = (b3Vec3){ 0.0f, -0.23941046f, 0.970918416f };
	planes[0].plane.offset = 0.390724182f;
	planes[0].pushLimit = FLT_MAX;
	planes[1].plane.normal = (b3Vec3){ 0.0f, 0.0f, 1.0f };
	planes[1].plane.offset = 1.49998093f;
	planes[1].pushLimit = FLT_MAX;

	b3Vec3 target = { -2.5390625f, 0.0f, -73.6880798f };

	planes[0].plane.offset -= b3Dot( planes[0].plane.normal, target );
	planes[1].plane.offset -= b3Dot( planes[1].plane.normal, target );
	target = b3Vec3_zero;

	b3PlaneSolverResult result = b3SolvePlanes( target, planes, 2 );

	ENSURE( result.iterationCount == 20 );

	return 0;
}

int MoverTest( void )
{
	RUN_SUBTEST( GamePlanes );
	RUN_SUBTEST( ParallelPlanes );

	return 0;
}
