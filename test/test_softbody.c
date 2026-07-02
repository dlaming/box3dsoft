// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <stdlib.h>

// Build a tetrahedral box lattice: (resolution+1)^3 grid points spanning +/- halfExtent,
// each cell split into 6 Kuhn tets sharing the main diagonal (conforming across cells).
// Returns malloc'd arrays the caller frees.
static void MakeBoxLattice( float halfExtent, int resolution, b3Vec3** outPositions, int* outCount, int** outTets,
							int* outTetCount )
{
	int np = resolution + 1;
	int count = np * np * np;
	float step = 2.0f * halfExtent / (float)resolution;

	b3Vec3* positions = (b3Vec3*)malloc( count * sizeof( b3Vec3 ) );
	for ( int z = 0; z < np; ++z )
	{
		for ( int y = 0; y < np; ++y )
		{
			for ( int x = 0; x < np; ++x )
			{
				positions[x + y * np + z * np * np] =
					( b3Vec3 ){ -halfExtent + x * step, -halfExtent + y * step, -halfExtent + z * step };
			}
		}
	}

	int cellCount = resolution * resolution * resolution;
	int tetCount = 6 * cellCount;
	int* tets = (int*)malloc( 4 * tetCount * sizeof( int ) );

	int t = 0;
	for ( int z = 0; z < resolution; ++z )
	{
		for ( int y = 0; y < resolution; ++y )
		{
			for ( int x = 0; x < resolution; ++x )
			{
				int c000 = x + y * np + z * np * np;
				int c100 = c000 + 1;
				int c010 = c000 + np;
				int c110 = c010 + 1;
				int c001 = c000 + np * np;
				int c101 = c001 + 1;
				int c011 = c001 + np;
				int c111 = c011 + 1;

				int kuhn[6][4] = {
					{ c000, c100, c110, c111 }, { c000, c110, c010, c111 }, { c000, c010, c011, c111 },
					{ c000, c011, c001, c111 }, { c000, c001, c101, c111 }, { c000, c101, c100, c111 },
				};

				for ( int k = 0; k < 6; ++k )
				{
					tets[4 * t + 0] = kuhn[k][0];
					tets[4 * t + 1] = kuhn[k][1];
					tets[4 * t + 2] = kuhn[k][2];
					tets[4 * t + 3] = kuhn[k][3];
					t += 1;
				}
			}
		}
	}

	*outPositions = positions;
	*outCount = count;
	*outTets = tets;
	*outTetCount = tetCount;
}

// Octahedron shell: 6 vertices, 8 triangles, outward winding.
static void MakeOctahedron( float radius, b3Vec3 positions[6], int triangles[24] )
{
	b3Vec3 p[6] = {
		{ radius, 0.0f, 0.0f }, { -radius, 0.0f, 0.0f }, { 0.0f, radius, 0.0f },
		{ 0.0f, -radius, 0.0f }, { 0.0f, 0.0f, radius }, { 0.0f, 0.0f, -radius },
	};
	int tri[24] = {
		0, 2, 4, 2, 1, 4, 1, 3, 4, 3, 0, 4, 2, 0, 5, 1, 2, 5, 3, 1, 5, 0, 3, 5,
	};

	for ( int i = 0; i < 6; ++i )
	{
		positions[i] = p[i];
	}
	for ( int i = 0; i < 24; ++i )
	{
		triangles[i] = tri[i];
	}
}

static int SoftBodyCreateDestroy( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3Vec3 positions[6];
	int triangles[24];
	MakeOctahedron( 0.5f, positions, triangles );

	b3SoftBodyDef def = b3DefaultSoftBodyDef();
	def.restPositions = positions;
	def.particleCount = 6;
	def.triangles = triangles;
	def.triangleCount = 8;
	def.origin = ( b3Pos ){ 1.0f, 2.0f, 3.0f };

	b3SoftBodyId id = b3CreateSoftBody( worldId, &def );
	ENSURE( b3SoftBody_IsValid( id ) );
	ENSURE( b3SoftBody_GetParticleCount( id ) == 6 );

	b3Pos centroid = b3SoftBody_GetCentroid( id );
	ENSURE_SMALL( (float)centroid.x - 1.0f, 1.0e-5f );
	ENSURE_SMALL( (float)centroid.y - 2.0f, 1.0e-5f );
	ENSURE_SMALL( (float)centroid.z - 3.0f, 1.0e-5f );

	b3Pos points[6];
	b3SoftBody_GetParticlePositions( id, points, 6 );
	ENSURE_SMALL( (float)points[0].x - 1.5f, 1.0e-5f );

	b3DestroySoftBody( id );
	ENSURE( b3SoftBody_IsValid( id ) == false );

	// id reuse gets a fresh generation
	b3SoftBodyId id2 = b3CreateSoftBody( worldId, &def );
	ENSURE( b3SoftBody_IsValid( id2 ) );
	ENSURE( b3SoftBody_IsValid( id ) == false );

	b3DestroyWorld( worldId );
	return 0;
}

// A jello cube dropped onto a static ground box must come to rest on the ground:
// not fall through, not explode, not creep away.
static int SoftBodyLatticeSettle( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	// static ground: top face at y = 0
	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 5.0f, 1.0f, 5.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3Vec3* positions;
	int* tets;
	int count, tetCount;
	float halfExtent = 0.25f;
	MakeBoxLattice( halfExtent, 2, &positions, &count, &tets, &tetCount );

	b3SoftBodyDef def = b3DefaultSoftBodyDef();
	def.restPositions = positions;
	def.particleCount = count;
	def.tets = tets;
	def.tetCount = tetCount;
	def.mass = 2.0f;
	def.origin = ( b3Pos ){ 0.0f, 1.0f, 0.0f };

	b3SoftBodyId id = b3CreateSoftBody( worldId, &def );
	ENSURE( b3SoftBody_IsValid( id ) );

	float dt = 1.0f / 60.0f;
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, dt, 4 );
	}

	b3Pos centroid = b3SoftBody_GetCentroid( id );

	// resting on the ground: the centroid sits roughly one half extent above it
	ENSURE( (float)centroid.y > 0.5f * halfExtent );
	ENSURE( (float)centroid.y < 3.0f * halfExtent );
	ENSURE( b3AbsFloat( (float)centroid.x ) < 0.5f );
	ENSURE( b3AbsFloat( (float)centroid.z ) < 0.5f );

	// every particle is finite and rests above the ground plane (particle centers stop about
	// one particle radius above it; anything at or below the plane means a vertex sank in)
	b3Pos* points = (b3Pos*)malloc( count * sizeof( b3Pos ) );
	b3SoftBody_GetParticlePositions( id, points, count );
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 p = b3ToVec3( points[i] );
		ENSURE( b3IsValidVec3( p ) );
		ENSURE( p.y > 0.0f );
	}
	free( points );

	// FEM mode keeps working from the settled state
	b3SoftBody_EnableFem( id, true );
	for ( int i = 0; i < 60; ++i )
	{
		b3World_Step( worldId, dt, 4 );
	}
	centroid = b3SoftBody_GetCentroid( id );
	ENSURE( (float)centroid.y > 0.5f * halfExtent );
	ENSURE( (float)centroid.y < 3.0f * halfExtent );

	free( positions );
	free( tets );
	b3DestroyWorld( worldId );
	return 0;
}

// A pressurized shell must hold its volume (not collapse or explode) while resting.
static int SoftBodyShellPressure( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 5.0f, 1.0f, 5.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3Vec3 positions[6];
	int triangles[24];
	float radius = 0.5f;
	MakeOctahedron( radius, positions, triangles );

	b3SoftBodyDef def = b3DefaultSoftBodyDef();
	def.restPositions = positions;
	def.particleCount = 6;
	def.triangles = triangles;
	def.triangleCount = 8;
	def.pressure = 1.0f;
	def.origin = ( b3Pos ){ 0.0f, 1.0f, 0.0f };

	b3SoftBodyId id = b3CreateSoftBody( worldId, &def );

	float dt = 1.0f / 60.0f;
	for ( int i = 0; i < 240; ++i )
	{
		b3World_Step( worldId, dt, 4 );
	}

	// still ball-sized and above ground: max particle distance from the centroid stays near rest
	b3Pos centroid = b3SoftBody_GetCentroid( id );
	ENSURE( (float)centroid.y > 0.1f );

	b3Pos points[6];
	b3SoftBody_GetParticlePositions( id, points, 6 );
	float maxR = 0.0f;
	for ( int i = 0; i < 6; ++i )
	{
		float r = b3Length( b3SubPos( points[i], centroid ) );
		maxR = b3MaxFloat( maxR, r );
	}
	ENSURE( maxR > 0.25f * radius );
	ENSURE( maxR < 3.0f * radius );

	b3DestroyWorld( worldId );
	return 0;
}

// A pinned particle must not move while the rest of the body hangs from it.
static int SoftBodyPinning( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	b3Vec3 positions[6];
	int triangles[24];
	MakeOctahedron( 0.5f, positions, triangles );

	b3SoftBodyDef def = b3DefaultSoftBodyDef();
	def.restPositions = positions;
	def.particleCount = 6;
	def.triangles = triangles;
	def.triangleCount = 8;
	def.origin = ( b3Pos ){ 0.0f, 2.0f, 0.0f };

	b3SoftBodyId id = b3CreateSoftBody( worldId, &def );

	// pin the +y pole (index 2)
	b3SoftBody_SetParticlePinned( id, 2, true );

	b3Pos before[6];
	b3SoftBody_GetParticlePositions( id, before, 6 );

	float dt = 1.0f / 60.0f;
	for ( int i = 0; i < 120; ++i )
	{
		b3World_Step( worldId, dt, 4 );
	}

	b3Pos after[6];
	b3SoftBody_GetParticlePositions( id, after, 6 );

	// pinned particle stayed, the opposite pole hangs below it
	ENSURE( b3Length( b3SubPos( after[2], before[2] ) ) < 1.0e-3f );
	ENSURE( (float)after[3].y < (float)after[2].y );

	b3DestroyWorld( worldId );
	return 0;
}

// Regression: resting contact must not sink. A resting particle reads as "touching" to the
// swept cast every substep (fraction-zero hit with an invalid zero normal); a bad response
// once let the safe anchor degrade onto the surface and vertices crept under the ground.
// Slam a shell down hard, then rest a long time, and require every particle to stay above
// the ground plane the whole way.
static int SoftBodyNoGroundSink( void )
{
	b3WorldDef worldDef = b3DefaultWorldDef();
	b3WorldId worldId = b3CreateWorld( &worldDef );

	{
		b3BodyDef bodyDef = b3DefaultBodyDef();
		bodyDef.position = ( b3Pos ){ 0.0f, -1.0f, 0.0f };
		b3BodyId groundId = b3CreateBody( worldId, &bodyDef );
		b3BoxHull box = b3MakeBoxHull( 5.0f, 1.0f, 5.0f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		b3CreateHullShape( groundId, &shapeDef, &box.base );
	}

	b3Vec3 positions[6];
	int triangles[24];
	MakeOctahedron( 0.3f, positions, triangles );

	b3SoftBodyDef def = b3DefaultSoftBodyDef();
	def.restPositions = positions;
	def.particleCount = 6;
	def.triangles = triangles;
	def.triangleCount = 8;
	def.origin = ( b3Pos ){ 0.0f, 0.6f, 0.0f };

	b3SoftBodyId id = b3CreateSoftBody( worldId, &def );

	// slam it into the ground
	b3SoftBody_ApplyLinearImpulse( id, ( b3Vec3 ){ 0.0f, -8.0f, 0.0f } );

	float dt = 1.0f / 60.0f;
	b3Pos points[6];
	for ( int step = 0; step < 600; ++step )
	{
		b3World_Step( worldId, dt, 4 );

		b3SoftBody_GetParticlePositions( id, points, 6 );
		for ( int i = 0; i < 6; ++i )
		{
			ENSURE( b3IsValidVec3( b3ToVec3( points[i] ) ) );
			ENSURE( (float)points[i].y > 0.0f );
		}
	}

	b3DestroyWorld( worldId );
	return 0;
}

int SoftBodyTest( void )
{
	RUN_SUBTEST( SoftBodyCreateDestroy );
	RUN_SUBTEST( SoftBodyLatticeSettle );
	RUN_SUBTEST( SoftBodyShellPressure );
	RUN_SUBTEST( SoftBodyPinning );
	RUN_SUBTEST( SoftBodyNoGroundSink );
	return 0;
}
