// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "softbody.h"

#include "core.h"
#include "physics_world.h"
#include "table.h"

#include <string.h>

b3SoftBodyDef b3DefaultSoftBodyDef( void )
{
	b3SoftBodyDef def = { 0 };
	def.origin = b3Pos_zero;
	def.mass = 1.0f;
	def.substepCount = 8;
	def.iterations = 1;
	def.edgeSoftness = 0.0f;
	def.volumeSoftness = 0.0f;
	def.pressure = 1.0f;
	def.enableShapeMatching = false;
	def.shapeStiffness = 0.2f;
	def.keepEdgesWithShapeMatching = false;
	def.enableFem = false;
	def.youngModulus = 1.0e5f;
	def.poissonRatio = 0.45f;
	def.damping = 1.0f;
	def.particleRadius = 0.0f;
	def.collisionRadiusScale = 1.1f;
	def.friction = 0.3f;
	def.maxParticleSpeed = 40.0f;
	def.enableSelfCollision = false;
	def.enableInterBodyCollision = false;
	def.collisionIterations = 2;
	def.selfExclusionScale = 2.0f;
	def.maxStretch = 2.0f;
	def.categoryBits = 1;
	def.maskBits = UINT64_MAX;
	def.internalValue = B3_SECRET_COOKIE;
	return def;
}

b3SoftBody* b3GetSoftBody( b3World* world, int softBodyId )
{
	b3SoftBody* body = b3Array_Get( world->softBodies, softBodyId );
	B3_ASSERT( body->softBodyId == softBodyId );
	return body;
}

b3SoftBody* b3GetSoftBodyFullId( b3World* world, b3SoftBodyId softBodyId )
{
	int id = softBodyId.index1 - 1;
	b3SoftBody* body = b3Array_Get( world->softBodies, id );
	B3_ASSERT( body->softBodyId == id && body->generation == softBodyId.generation );
	return body;
}

static b3SoftBodyId b3MakeSoftBodyId( b3World* world, const b3SoftBody* body )
{
	b3SoftBodyId id = { body->softBodyId + 1, world->worldId, body->generation };
	return id;
}

// Add the unique undirected edge (i1, i2) to the edge array.
static void b3AddSoftEdge( b3HashSet* set, b3SoftEdge* edges, int* edgeCount, int i1, int i2, const b3Vec3* restPositions )
{
	if ( i1 == i2 )
	{
		return;
	}

	int lo = b3MinInt( i1, i2 );
	int hi = b3MaxInt( i1, i2 );
	uint64_t key = ( (uint64_t)lo << 32 ) | (uint64_t)(uint32_t)hi;
	if ( b3AddKey( set, key ) )
	{
		// already present
		return;
	}

	b3SoftEdge* edge = edges + *edgeCount;
	edge->i1 = i1;
	edge->i2 = i2;
	edge->restLength = b3Distance( restPositions[i1], restPositions[i2] );
	edge->lambda = 0.0f;
	*edgeCount += 1;
}

static float b3SignedTetVolume( b3Vec3 a, b3Vec3 b, b3Vec3 c, b3Vec3 d )
{
	return b3Dot( b3Sub( b, a ), b3Cross( b3Sub( c, a ), b3Sub( d, a ) ) ) / 6.0f;
}

b3SoftBodyId b3CreateSoftBody( b3WorldId worldId, const b3SoftBodyDef* def )
{
	B3_CHECK_DEF( def );
	B3_ASSERT( def->particleCount > 0 && def->restPositions != NULL );
	B3_ASSERT( def->triangleCount > 0 || def->tetCount > 0 );
	B3_ASSERT( b3IsValidFloat( def->mass ) && def->mass > 0.0f );

	b3World* world = b3GetUnlockedWorldFromId( worldId );
	if ( world == NULL )
	{
		return b3_nullSoftBodyId;
	}

	int softBodyId = b3AllocId( &world->softBodyIdPool );
	if ( softBodyId == world->softBodies.count )
	{
		b3Array_Push( world->softBodies, ( b3SoftBody ){ 0 } );
	}

	b3SoftBody* body = b3Array_Get( world->softBodies, softBodyId );
	uint16_t generation = (uint16_t)( body->generation + 1 );
	memset( body, 0, sizeof( b3SoftBody ) );
	body->softBodyId = softBodyId;
	body->generation = generation;
	body->userData = def->userData;

	int count = def->particleCount;
	body->particleCount = count;

	body->p = (b3Vec3*)b3Alloc( count * sizeof( b3Vec3 ) );
	body->p0 = (b3Vec3*)b3Alloc( count * sizeof( b3Vec3 ) );
	body->safe = (b3Vec3*)b3Alloc( count * sizeof( b3Vec3 ) );
	body->velocity = (b3Vec3*)b3AllocZeroed( count * sizeof( b3Vec3 ) );
	body->invMass = (float*)b3Alloc( count * sizeof( float ) );
	body->rest = (b3Vec3*)b3Alloc( count * sizeof( b3Vec3 ) );
	body->restRadius = (float*)b3Alloc( count * sizeof( float ) );
	body->volumeGradient = (b3Vec3*)b3Alloc( count * sizeof( b3Vec3 ) );

	// Rest positions about the rest centroid. The body origin carries the centroid so the
	// particle floats stay small.
	b3Vec3 restCentroid = b3Vec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		restCentroid = b3Add( restCentroid, def->restPositions[i] );
	}
	restCentroid = b3MulSV( 1.0f / (float)count, restCentroid );

	body->boundRadius = 0.0f;
	float invParticleMass = (float)count / def->mass;
	body->particleMass = def->mass / (float)count;
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 r = b3Sub( def->restPositions[i], restCentroid );
		body->rest[i] = r;
		body->restRadius[i] = b3Length( r );
		body->boundRadius = b3MaxFloat( body->boundRadius, body->restRadius[i] );
		body->p[i] = r;
		body->p0[i] = r;
		body->safe[i] = r;
		body->invMass[i] = invParticleMass;
	}

	body->origin = b3OffsetPos( def->origin, restCentroid );

	// Copy topology
	if ( def->triangleCount > 0 )
	{
		B3_ASSERT( def->triangles != NULL );
		body->triangleCount = def->triangleCount;
		body->triangles = (int32_t*)b3Alloc( 3 * def->triangleCount * sizeof( int32_t ) );
		memcpy( body->triangles, def->triangles, 3 * def->triangleCount * sizeof( int32_t ) );
	}

	if ( def->tetCount > 0 )
	{
		B3_ASSERT( def->tets != NULL );
		body->tetCount = def->tetCount;
		body->tets = (b3SoftTet*)b3AllocZeroed( def->tetCount * sizeof( b3SoftTet ) );
		for ( int t = 0; t < def->tetCount; ++t )
		{
			b3SoftTet* tet = body->tets + t;
			tet->i[0] = def->tets[4 * t + 0];
			tet->i[1] = def->tets[4 * t + 1];
			tet->i[2] = def->tets[4 * t + 2];
			tet->i[3] = def->tets[4 * t + 3];
			B3_ASSERT( tet->i[0] < count && tet->i[1] < count && tet->i[2] < count && tet->i[3] < count );

			b3Vec3 x0 = body->rest[tet->i[0]];
			b3Vec3 e1 = b3Sub( body->rest[tet->i[1]], x0 );
			b3Vec3 e2 = b3Sub( body->rest[tet->i[2]], x0 );
			b3Vec3 e3 = b3Sub( body->rest[tet->i[3]], x0 );
			tet->restVolume = b3Dot( e1, b3Cross( e2, e3 ) ) / 6.0f;

			// Dm^-1 for the deformation gradient F = Ds * Dm^-1
			b3Matrix3 dm = { e1, e2, e3 };
			b3Matrix3 dmInv = b3InvertMatrix( dm );
			tet->dmInvC0 = dmInv.cx;
			tet->dmInvC1 = dmInv.cy;
			tet->dmInvC2 = dmInv.cz;
		}
	}

	// Unique edges from triangles and tets
	int maxEdges = 3 * def->triangleCount + 6 * def->tetCount;
	body->edges = (b3SoftEdge*)b3Alloc( maxEdges * sizeof( b3SoftEdge ) );
	body->edgeCount = 0;
	{
		b3HashSet edgeSet = b3CreateSet( 2 * maxEdges );

		for ( int t = 0; t < def->triangleCount; ++t )
		{
			int a = def->triangles[3 * t + 0];
			int b = def->triangles[3 * t + 1];
			int c = def->triangles[3 * t + 2];
			B3_ASSERT( a < count && b < count && c < count );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, a, b, body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, b, c, body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, c, a, body->rest );
		}

		for ( int t = 0; t < def->tetCount; ++t )
		{
			const int32_t* i = body->tets[t].i;
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[0], i[1], body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[0], i[2], body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[0], i[3], body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[1], i[2], body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[1], i[3], body->rest );
			b3AddSoftEdge( &edgeSet, body->edges, &body->edgeCount, i[2], i[3], body->rest );
		}

		b3DestroySet( &edgeSet );
	}

	// Rest volume of the enclosed surface
	body->restVolume = 0.0f;
	for ( int t = 0; t < body->triangleCount; ++t )
	{
		b3Vec3 a = body->rest[body->triangles[3 * t + 0]];
		b3Vec3 b = body->rest[body->triangles[3 * t + 1]];
		b3Vec3 c = body->rest[body->triangles[3 * t + 2]];
		body->restVolume += b3Dot( a, b3Cross( b, c ) ) / 6.0f;
	}

	// Collision radius from the mean rest edge length: neighbouring particles just touch.
	float meanEdge = 0.0f;
	for ( int k = 0; k < body->edgeCount; ++k )
	{
		meanEdge += body->edges[k].restLength;
	}
	meanEdge = body->edgeCount > 0 ? meanEdge / (float)body->edgeCount : 0.0f;
	float radiusScale = def->collisionRadiusScale > 0.0f ? def->collisionRadiusScale : 1.0f;
	body->collisionRadius = 0.5f * meanEdge * radiusScale;
	body->particleRadius = def->particleRadius > 0.0f ? def->particleRadius : body->collisionRadius;

	body->substepCount = b3MaxInt( def->substepCount, 1 );
	body->iterations = b3MaxInt( def->iterations, 1 );
	body->edgeSoftness = b3ClampFloat( def->edgeSoftness, 0.0f, 1.0f );
	body->volumeSoftness = b3ClampFloat( def->volumeSoftness, 0.0f, 1.0f );
	body->pressure = def->pressure;
	body->enableShapeMatching = def->enableShapeMatching;
	body->shapeStiffness = b3ClampFloat( def->shapeStiffness, 0.0f, 1.0f );
	body->keepEdgesWithShapeMatching = def->keepEdgesWithShapeMatching;
	body->enableFem = def->enableFem;
	body->youngModulus = b3MaxFloat( def->youngModulus, 1.0e-3f );
	body->poissonRatio = b3ClampFloat( def->poissonRatio, 0.0f, 0.49f );
	body->damping = b3MaxFloat( def->damping, 0.0f );
	body->friction = b3ClampFloat( def->friction, 0.0f, 1.0f );
	body->maxParticleSpeed = b3MaxFloat( def->maxParticleSpeed, 1.0f );
	body->enableSelfCollision = def->enableSelfCollision;
	body->enableInterBodyCollision = def->enableInterBodyCollision;
	body->collisionIterations = b3ClampInt( def->collisionIterations, 1, 8 );
	body->selfExclusionScale = b3MaxFloat( def->selfExclusionScale, 1.0f );
	body->maxStretch = def->maxStretch;
	body->categoryBits = def->categoryBits;
	body->maskBits = def->maskBits;

	body->shapeRotation = b3Quat_identity;
	body->volumeLambda = 0.0f;

	body->traceCandidates = b3CreateBitSet( count );
	body->worldNear = false;
	body->forceAllTrace = false;

	return b3MakeSoftBodyId( world, body );
}

void b3FreeSoftBodyStorage( b3SoftBody* body )
{
	int count = body->particleCount;
	if ( count == 0 )
	{
		return;
	}

	b3Free( body->p, count * sizeof( b3Vec3 ) );
	b3Free( body->p0, count * sizeof( b3Vec3 ) );
	b3Free( body->safe, count * sizeof( b3Vec3 ) );
	b3Free( body->velocity, count * sizeof( b3Vec3 ) );
	b3Free( body->invMass, count * sizeof( float ) );
	b3Free( body->rest, count * sizeof( b3Vec3 ) );
	b3Free( body->restRadius, count * sizeof( float ) );
	b3Free( body->volumeGradient, count * sizeof( b3Vec3 ) );

	int maxEdges = 3 * body->triangleCount + 6 * body->tetCount;
	b3Free( body->edges, maxEdges * sizeof( b3SoftEdge ) );

	if ( body->triangleCount > 0 )
	{
		b3Free( body->triangles, 3 * body->triangleCount * sizeof( int32_t ) );
	}

	if ( body->tetCount > 0 )
	{
		b3Free( body->tets, body->tetCount * sizeof( b3SoftTet ) );
	}

	b3DestroyBitSet( &body->traceCandidates );

	uint16_t generation = body->generation;
	int softBodyId = body->softBodyId;
	memset( body, 0, sizeof( b3SoftBody ) );
	body->generation = generation;
	body->softBodyId = B3_NULL_INDEX;
	B3_UNUSED( softBodyId );
}

void b3DestroySoftBody( b3SoftBodyId softBodyId )
{
	b3World* world = b3GetUnlockedWorld( softBodyId.world0 );
	if ( world == NULL )
	{
		return;
	}

	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	int id = body->softBodyId;
	b3FreeSoftBodyStorage( body );
	b3FreeId( &world->softBodyIdPool, id );
}

bool b3SoftBody_IsValid( b3SoftBodyId id )
{
	if ( B3_IS_NULL( id ) )
	{
		return false;
	}

	b3World* world = b3GetWorld( id.world0 );
	if ( world == NULL )
	{
		return false;
	}

	int softBodyId = id.index1 - 1;
	if ( softBodyId < 0 || world->softBodies.count <= softBodyId )
	{
		return false;
	}

	b3SoftBody* body = world->softBodies.data + softBodyId;
	return body->softBodyId == softBodyId && body->generation == id.generation;
}

int b3SoftBody_GetParticleCount( b3SoftBodyId softBodyId )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	return body->particleCount;
}

void b3SoftBody_GetParticlePositions( b3SoftBodyId softBodyId, b3Pos* positions, int capacity )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	int n = b3MinInt( body->particleCount, capacity );
	for ( int i = 0; i < n; ++i )
	{
		positions[i] = b3OffsetPos( body->origin, body->p[i] );
	}
}

static b3Vec3 b3SoftBodyLocalCentroid( const b3SoftBody* body )
{
	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < body->particleCount; ++i )
	{
		c = b3Add( c, body->p[i] );
	}
	return b3MulSV( 1.0f / (float)body->particleCount, c );
}

b3Pos b3SoftBody_GetCentroid( b3SoftBodyId softBodyId )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	return b3OffsetPos( body->origin, b3SoftBodyLocalCentroid( body ) );
}

void b3SoftBody_SetPosition( b3SoftBodyId softBodyId, b3Pos position )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );

	b3Vec3 c = b3SoftBodyLocalCentroid( body );
	for ( int i = 0; i < body->particleCount; ++i )
	{
		body->p[i] = b3Sub( body->p[i], c );
		body->p0[i] = body->p[i];
		body->safe[i] = body->p[i];
		body->velocity[i] = b3Vec3_zero;
	}

	body->origin = position;
}

void b3SoftBody_ApplyLinearImpulse( b3SoftBodyId softBodyId, b3Vec3 impulse )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );

	// Impulse on the whole body: dv = J / M, identical for every particle.
	b3Vec3 dv = b3MulSV( 1.0f / ( body->particleMass * (float)body->particleCount ), impulse );
	for ( int i = 0; i < body->particleCount; ++i )
	{
		if ( body->invMass[i] > 0.0f )
		{
			body->velocity[i] = b3Add( body->velocity[i], dv );
		}
	}
}

void b3SoftBody_SetParticlePinned( b3SoftBodyId softBodyId, int particleIndex, bool pinned )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	B3_ASSERT( 0 <= particleIndex && particleIndex < body->particleCount );
	body->invMass[particleIndex] = pinned ? 0.0f : 1.0f / body->particleMass;
	if ( pinned )
	{
		body->velocity[particleIndex] = b3Vec3_zero;
	}
}

void b3SoftBody_SetParticlePosition( b3SoftBodyId softBodyId, int particleIndex, b3Pos position )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	B3_ASSERT( 0 <= particleIndex && particleIndex < body->particleCount );
	b3Vec3 local = b3SubPos( position, body->origin );
	body->p[particleIndex] = local;
	body->p0[particleIndex] = local;
	body->safe[particleIndex] = local;
	body->velocity[particleIndex] = b3Vec3_zero;
}

void b3SoftBody_SetPressure( b3SoftBodyId softBodyId, float pressure )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->pressure = pressure;
}

void b3SoftBody_EnableShapeMatching( b3SoftBodyId softBodyId, bool flag )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->enableShapeMatching = flag;
}

void b3SoftBody_EnableFem( b3SoftBodyId softBodyId, bool flag )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->enableFem = flag;
}

void b3SoftBody_SetSoftness( b3SoftBodyId softBodyId, float edgeSoftness, float volumeSoftness )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->edgeSoftness = b3ClampFloat( edgeSoftness, 0.0f, 1.0f );
	body->volumeSoftness = b3ClampFloat( volumeSoftness, 0.0f, 1.0f );
}

void b3SoftBody_SetMaterial( b3SoftBodyId softBodyId, float youngModulus, float poissonRatio )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->youngModulus = b3MaxFloat( youngModulus, 1.0e-3f );
	body->poissonRatio = b3ClampFloat( poissonRatio, 0.0f, 0.49f );
}

void b3SoftBody_SetUserData( b3SoftBodyId softBodyId, void* userData )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	body->userData = userData;
}

void* b3SoftBody_GetUserData( b3SoftBodyId softBodyId )
{
	b3World* world = b3GetWorld( softBodyId.world0 );
	b3SoftBody* body = b3GetSoftBodyFullId( world, softBodyId );
	return body->userData;
}
