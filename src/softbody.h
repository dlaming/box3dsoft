// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "bitset.h"
#include "container.h"

#include "box3d/softbody.h"
#include "box3d/types.h"

typedef struct b3World b3World;

// A soft body particle edge (distance constraint).
typedef struct b3SoftEdge
{
	int32_t i1, i2;
	float restLength;
	float lambda;
} b3SoftEdge;

// A tetrahedron with precomputed rest data for the per-tet volume and Neo-Hookean constraints.
typedef struct b3SoftTet
{
	int32_t i[4];

	// Signed rest volume (per-tet volume constraint target).
	float restVolume;

	// Columns of the inverse rest edge matrix Dm^-1 (Neo-Hookean deformation gradient).
	b3Vec3 dmInvC0, dmInvC1, dmInvC2;

	// Accumulated XPBD lambdas, reset each substep.
	float lambdaVolume;
	float lambdaDev;
	float lambdaHyd;
} b3SoftTet;

// Particle-based deformable body solved with XPBD. Particle positions are floats relative to
// `origin` (a full precision world position); the origin is re-centered onto the particle
// centroid every step so float precision holds anywhere in a large world.
typedef struct b3SoftBody
{
	void* userData;

	// World position of the local frame. Particle world position = origin + p[i].
	b3Pos origin;

	// Particle state, all counts = particleCount.
	b3Vec3* p;			// positions (local to origin)
	b3Vec3* p0;			// positions at the start of the current substep
	b3Vec3* velocity;
	float* invMass;
	b3Vec3* rest;		// rest positions about the rest centroid (shape matching + self exclusion)
	float* restRadius;	// rest distance from the rest centroid (max stretch clamp)

	// Scratch for the enclosed volume constraint gradient, particleCount entries.
	b3Vec3* volumeGradient;

	// Unique edges from the surface triangles and/or tets.
	b3SoftEdge* edges;
	int edgeCount;

	// Surface triangles (three indices each), used by the enclosed volume constraint.
	int32_t* triangles;
	int triangleCount;

	b3SoftTet* tets;
	int tetCount;

	int particleCount;

	// Enclosed volume constraint state.
	float restVolume;
	float volumeLambda;

	// Warm-started best-fit rotation for shape matching.
	b3Quat shapeRotation;

	// Parameters (see b3SoftBodyDef).
	float particleMass;
	int substepCount;
	int iterations;
	float edgeSoftness;
	float volumeSoftness;
	float pressure;
	float shapeStiffness;
	float youngModulus;
	float poissonRatio;
	float damping;
	float particleRadius;
	float collisionRadius; // self/inter particle collision radius
	float friction;
	float maxParticleSpeed;
	float selfExclusionScale;
	float maxStretch;
	int collisionIterations;
	uint64_t categoryBits;
	uint64_t maskBits;
	bool enableShapeMatching;
	bool keepEdgesWithShapeMatching;
	bool enableFem;
	bool enableSelfCollision;
	bool enableInterBodyCollision;

	// Derived body size: max rest distance from the rest centroid.
	float boundRadius;

	// Per-step world collision culling (sized particleCount).
	b3BitSet traceCandidates;
	bool worldNear;
	bool forceAllTrace;

	int softBodyId;
	uint16_t generation;
} b3SoftBody;

b3DeclareArray( b3SoftBody );

b3SoftBody* b3GetSoftBody( b3World* world, int softBodyId );
b3SoftBody* b3GetSoftBodyFullId( b3World* world, b3SoftBodyId softBodyId );

// Destroy internal storage without touching the id pool (world teardown).
void b3FreeSoftBodyStorage( b3SoftBody* body );

// Step all soft bodies. Called at the end of b3World_Step while the world is locked.
void b3SolveSoftBodies( b3World* world, float dt );
