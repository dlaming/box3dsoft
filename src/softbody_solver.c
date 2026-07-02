// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

// XPBD soft body solver. Ported from a proven game implementation:
// - small steps (many substeps, few iterations) per Macklin/Mueller 2016
// - compliance is scaled by each constraint's own gradient magnitude so the 0-1 softness
//   knobs behave consistently across mesh scale, constraint type, and substep count
// - stable Neo-Hookean FEM per Macklin & Mueller 2021 with the rest-stable hydrostatic
//   target gamma = 1 + mu/lambda (Smith et al. 2018)
// - shape matching per Mueller 2005 with the robust rotation extraction of Mueller 2016
// - world collision is per-substep swept sphere casting against the broadphase with a
//   "last safe position" anchor for thin geometry, plus bounded two-way rigid impulses
// - self/inter-body collision through a flat spatial hash with a volume depenetration net

#include "softbody.h"

#include "arena_allocator.h"
#include "body.h"
#include "core.h"
#include "physics_world.h"
#include "shape.h"
#include "solver_set.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <math.h>
#include <string.h>

// How soft a softness knob of 1 is: corrections are reduced by roughly 1 + this.
#define B3_SOFT_SCALE 10.0f

// Distance the safe anchor is nudged off a surface after a resolved contact. Must exceed the
// shape cast tolerance (a fraction of B3_LINEAR_SLOP) or the anchor itself reads as touching
// and the recovery cast degenerates to a zero-normal overlap hit.
#define B3_SOFT_SAFE_MARGIN ( 2.0f * B3_LINEAR_SLOP )

// Map normalized softness [0,1] to an XPBD compliance scaled by the constraint's own
// gradient measure (sum of w * |gradC|^2). Zero softness is rigid (plain PBD).
static inline float b3SoftAlpha( float softness, float wGrad )
{
	return softness <= 0.0f ? 0.0f : softness * B3_SOFT_SCALE * wGrad;
}

// ---------------------------------------------------------------------------
// World collision: swept sphere cast against the broadphase (internal, world is locked)
// ---------------------------------------------------------------------------

typedef struct b3SoftCastContext
{
	b3World* world;

	// Query origin carried at full precision. Cast inputs/outputs are relative to this.
	b3Pos origin;

	b3ShapeCastInput input;
	uint64_t categoryBits;
	uint64_t maskBits;

	bool hit;
	float fraction;
	b3Vec3 normal;
	b3Vec3 point; // origin relative
	int shapeId;
} b3SoftCastContext;

static float b3SoftCastCallback( const b3BoxCastInput* input, int proxyId, uint64_t userData, void* context )
{
	B3_UNUSED( proxyId );

	int shapeId = (int)userData;
	b3SoftCastContext* ctx = (b3SoftCastContext*)context;
	b3World* world = ctx->world;

	b3Shape* shape = b3Array_Get( world->shapes, shapeId );
	if ( shape->sensorIndex != B3_NULL_INDEX )
	{
		return input->maxFraction;
	}

	b3QueryFilter queryFilter = { ctx->categoryBits, ctx->maskBits, 0, NULL };
	if ( b3ShouldQueryCollide( &shape->filter, &queryFilter ) == false )
	{
		return input->maxFraction;
	}

	b3ShapeCastInput localInput = ctx->input;
	localInput.maxFraction = input->maxFraction;

	b3Body* body = b3Array_Get( world->bodies, shape->bodyId );
	b3Transform transform = b3ToRelativeTransform( b3GetBodyTransformQuick( world, body ), ctx->origin );

	b3CastOutput output = b3ShapeCastShape( shape, transform, &localInput );

	if ( output.hit && output.fraction <= ctx->fraction )
	{
		ctx->hit = true;
		ctx->fraction = output.fraction;
		ctx->normal = output.normal;
		ctx->point = output.point;
		ctx->shapeId = shapeId;
		return output.fraction;
	}

	return input->maxFraction;
}

// Sweep a sphere of `radius` from body-local `from` along `translation`. Results are body-local.
// A cast that starts touching or penetrating returns hit at fraction zero with a ZERO normal;
// callers must not use that frame and instead recover with a cast that starts clear (the safe
// anchor sits B3_SOFT_SAFE_MARGIN off the surface, which exceeds the cast tolerance, exactly so
// it can serve as that clear start). A resting particle sits one linear slop encroached after
// every resolved contact and therefore reads as touching every substep: resting contact is
// intentionally resolved through the recovery path each substep.
static b3SoftCastContext b3SoftBodySphereCast( b3World* world, const b3SoftBody* body, b3Vec3 from, b3Vec3 translation,
											   float radius, bool canEncroach )
{
	b3SoftCastContext ctx = { 0 };
	ctx.world = world;
	ctx.origin = b3OffsetPos( body->origin, from );
	ctx.categoryBits = body->categoryBits;
	ctx.maskBits = body->maskBits;
	ctx.fraction = 1.0f;

	static const b3Vec3 zero = { 0.0f, 0.0f, 0.0f };
	ctx.input.proxy.points = &zero;
	ctx.input.proxy.count = 1;
	ctx.input.proxy.radius = radius;
	ctx.input.translation = translation;
	ctx.input.maxFraction = 1.0f;
	ctx.input.canEncroach = canEncroach;

	b3AABB localBox = b3MakeAABB( &zero, 1, radius );
	b3BoxCastInput treeInput = { b3OffsetAABB( localBox, ctx.origin ), translation, 1.0f };

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		b3DynamicTree_BoxCast( world->broadPhase.trees + i, &treeInput, body->maskBits, false, b3SoftCastCallback, &ctx );
		if ( ctx.fraction == 0.0f )
		{
			break;
		}
		treeInput.maxFraction = ctx.fraction;
	}

	// convert the hit point to body local
	if ( ctx.hit )
	{
		ctx.point = b3Add( from, ctx.point );
	}

	return ctx;
}

// ---------------------------------------------------------------------------
// Per-step world collision culling
// ---------------------------------------------------------------------------

typedef struct b3SoftCullContext
{
	b3World* world;
	const b3DynamicTree* tree;
	uint64_t categoryBits;
	uint64_t maskBits;
	b3AABB* bounds;
	int count;
	int capacity;
	bool overflow;
} b3SoftCullContext;

static bool b3SoftCullCallback( int proxyId, uint64_t userData, void* context )
{
	b3SoftCullContext* ctx = (b3SoftCullContext*)context;

	int shapeId = (int)userData;
	b3Shape* shape = b3Array_Get( ctx->world->shapes, shapeId );
	if ( shape->sensorIndex != B3_NULL_INDEX )
	{
		return true;
	}

	b3QueryFilter queryFilter = { ctx->categoryBits, ctx->maskBits, 0, NULL };
	if ( b3ShouldQueryCollide( &shape->filter, &queryFilter ) == false )
	{
		return true;
	}

	if ( ctx->count == ctx->capacity )
	{
		ctx->overflow = true;
		return false;
	}

	ctx->bounds[ctx->count] = b3DynamicTree_GetAABB( ctx->tree, proxyId );
	ctx->count += 1;
	return true;
}

// Once per step: classify which particles could touch world geometry this step so the
// per-substep sweep only casts those. The band covers a particle's whole-step travel plus
// its radius, so any particle that could reach a collider is flagged.
static void b3SoftBodyPrepareWorldCollision( b3World* world, b3SoftBody* body, float dt )
{
	body->worldNear = false;
	body->forceAllTrace = false;

	int count = body->particleCount;
	float band = body->particleRadius + body->maxParticleSpeed * dt + body->collisionRadius + 0.05f;

	// body bounds in local space
	b3Vec3 lower = body->p[0];
	b3Vec3 upper = body->p[0];
	for ( int i = 1; i < count; ++i )
	{
		lower = b3Min( lower, body->p[i] );
		upper = b3Max( upper, body->p[i] );
	}
	b3Vec3 grow = { band, band, band };
	b3AABB localBox = { b3Sub( lower, grow ), b3Add( upper, grow ) };
	b3AABB queryBox = b3OffsetAABB( localBox, body->origin );

	enum
	{
		maxBounds = 256
	};
	b3AABB* bounds = (b3AABB*)b3StackAlloc( &world->stack, maxBounds * sizeof( b3AABB ), "soft cull bounds" );

	b3SoftCullContext ctx = { 0 };
	ctx.world = world;
	ctx.categoryBits = body->categoryBits;
	ctx.maskBits = body->maskBits;
	ctx.bounds = bounds;
	ctx.capacity = maxBounds;

	for ( int i = 0; i < b3_bodyTypeCount; ++i )
	{
		ctx.tree = world->broadPhase.trees + i;
		b3DynamicTree_Query( world->broadPhase.trees + i, queryBox, body->maskBits, false, b3SoftCullCallback, &ctx );
		if ( ctx.overflow )
		{
			break;
		}
	}

	if ( ctx.count == 0 && ctx.overflow == false )
	{
		// nothing anywhere near the body this step
		b3StackFree( &world->stack, bounds );
		return;
	}

	body->worldNear = true;

	if ( ctx.overflow )
	{
		// too many nearby shapes to classify per particle; sweep everyone (always safe)
		body->forceAllTrace = true;
		b3StackFree( &world->stack, bounds );
		return;
	}

	b3SetBitCountAndClear( &body->traceCandidates, count );
	for ( int i = 0; i < count; ++i )
	{
		b3Vec3 pw = b3ToVec3( b3OffsetPos( body->origin, body->p[i] ) );
		for ( int k = 0; k < ctx.count; ++k )
		{
			b3AABB fat = ctx.bounds[k];
			fat.lowerBound = b3Sub( fat.lowerBound, grow );
			fat.upperBound = b3Add( fat.upperBound, grow );
			if ( fat.lowerBound.x <= pw.x && pw.x <= fat.upperBound.x && fat.lowerBound.y <= pw.y && pw.y <= fat.upperBound.y &&
				 fat.lowerBound.z <= pw.z && pw.z <= fat.upperBound.z )
			{
				b3SetBit( &body->traceCandidates, i );
				break;
			}
		}
	}

	b3StackFree( &world->stack, bounds );
}

// ---------------------------------------------------------------------------
// Integration and damping
// ---------------------------------------------------------------------------

// Bleed velocity toward the body's rigid motion (center-of-mass velocity plus rotation from
// L = I w), preserving linear and angular momentum. Removes internal jiggle only; falling
// and rolling are untouched.
static void b3SoftBodyApplyRigidDamping( b3SoftBody* body, float k )
{
	if ( k <= 0.0f )
	{
		return;
	}

	int n = body->particleCount;
	b3Vec3* p = body->p;
	b3Vec3* v = body->velocity;

	b3Vec3 com = b3Vec3_zero;
	b3Vec3 vcm = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		com = b3Add( com, p[i] );
		vcm = b3Add( vcm, v[i] );
	}
	float invN = 1.0f / (float)n;
	com = b3MulSV( invN, com );
	vcm = b3MulSV( invN, vcm );

	// angular momentum and inertia tensor about the com (unit particle mass; mass cancels)
	b3Vec3 L = b3Vec3_zero;
	float ixx = 0.0f, iyy = 0.0f, izz = 0.0f, ixy = 0.0f, ixz = 0.0f, iyz = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 r = b3Sub( p[i], com );
		L = b3Add( L, b3Cross( r, v[i] ) );
		ixx += r.y * r.y + r.z * r.z;
		iyy += r.x * r.x + r.z * r.z;
		izz += r.x * r.x + r.y * r.y;
		ixy -= r.x * r.y;
		ixz -= r.x * r.z;
		iyz -= r.y * r.z;
	}

	// omega = I^-1 L via cofactors of the symmetric 3x3
	float m00 = iyy * izz - iyz * iyz;
	float m01 = ixz * iyz - ixy * izz;
	float m02 = ixy * iyz - ixz * iyy;
	float m11 = ixx * izz - ixz * ixz;
	float m12 = ixy * ixz - ixx * iyz;
	float m22 = ixx * iyy - ixy * ixy;
	float det = ixx * m00 + ixy * m01 + ixz * m02;

	b3Vec3 omega = b3Vec3_zero;
	if ( b3AbsFloat( det ) > 1.0e-6f )
	{
		float inv = 1.0f / det;
		omega.x = ( m00 * L.x + m01 * L.y + m02 * L.z ) * inv;
		omega.y = ( m01 * L.x + m11 * L.y + m12 * L.z ) * inv;
		omega.z = ( m02 * L.x + m12 * L.y + m22 * L.z ) * inv;
	}

	for ( int i = 0; i < n; ++i )
	{
		if ( body->invMass[i] == 0.0f )
		{
			continue;
		}
		b3Vec3 rigid = b3Add( vcm, b3Cross( omega, b3Sub( p[i], com ) ) );
		v[i] = b3Add( v[i], b3MulSV( k, b3Sub( rigid, v[i] ) ) );
	}
}

static void b3SoftBodyIntegrate( b3SoftBody* body, b3Vec3 gravity, float sdt )
{
	int n = body->particleCount;

	// external acceleration is added undamped so damping never weakens gravity
	for ( int i = 0; i < n; ++i )
	{
		if ( body->invMass[i] > 0.0f )
		{
			body->velocity[i] = b3MulAdd( body->velocity[i], sdt, gravity );
		}
	}

	b3SoftBodyApplyRigidDamping( body, b3MinFloat( body->damping * sdt, 1.0f ) );

	for ( int i = 0; i < n; ++i )
	{
		body->p0[i] = body->p[i];
		if ( body->invMass[i] > 0.0f )
		{
			body->p[i] = b3MulAdd( body->p[i], sdt, body->velocity[i] );
		}
	}
}

// ---------------------------------------------------------------------------
// Constraints
// ---------------------------------------------------------------------------

static void b3SoftBodyBeginSubstep( b3SoftBody* body )
{
	for ( int k = 0; k < body->edgeCount; ++k )
	{
		body->edges[k].lambda = 0.0f;
	}

	for ( int t = 0; t < body->tetCount; ++t )
	{
		body->tets[t].lambdaVolume = 0.0f;
		body->tets[t].lambdaDev = 0.0f;
		body->tets[t].lambdaHyd = 0.0f;
	}

	body->volumeLambda = 0.0f;
}

static bool b3SoftBodyEdgesActive( const b3SoftBody* body )
{
	if ( body->tetCount > 0 )
	{
		// solids: edge springs are replaced by the FEM model
		return body->enableFem == false;
	}

	// shells: shape matching replaces the structural edges unless explicitly kept
	return body->enableShapeMatching == false || body->keepEdgesWithShapeMatching;
}

static void b3SoftBodySolveEdges( b3SoftBody* body )
{
	b3Vec3* p = body->p;
	const float* w = body->invMass;
	float softness = body->edgeSoftness;

	for ( int k = 0; k < body->edgeCount; ++k )
	{
		b3SoftEdge* edge = body->edges + k;
		int a = edge->i1;
		int b = edge->i2;

		b3Vec3 d = b3Sub( p[a], p[b] );
		float len = b3Length( d );
		if ( len < 1.0e-7f )
		{
			continue;
		}

		float wsum = w[a] + w[b]; // edge gradients are unit vectors
		if ( wsum <= 0.0f )
		{
			continue;
		}

		b3Vec3 nrm = b3MulSV( 1.0f / len, d );
		float c = len - edge->restLength;
		float alpha = b3SoftAlpha( softness, wsum );
		float dLambda = ( -c - alpha * edge->lambda ) / ( wsum + alpha );
		edge->lambda += dLambda;

		p[a] = b3MulAdd( p[a], w[a] * dLambda, nrm );
		p[b] = b3MulSub( p[b], w[b] * dLambda, nrm );
	}
}

// Single enclosed-volume (gas pressure) constraint over the surface triangles.
static void b3SoftBodySolveEnclosedVolume( b3SoftBody* body )
{
	if ( body->triangleCount == 0 || body->restVolume == 0.0f )
	{
		return;
	}

	int n = body->particleCount;
	b3Vec3* p = body->p;
	const float* w = body->invMass;
	b3Vec3* grad = body->volumeGradient;

	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		c = b3Add( c, p[i] );
	}
	c = b3MulSV( 1.0f / (float)n, c );

	memset( grad, 0, n * sizeof( b3Vec3 ) );
	float volume = 0.0f;
	for ( int k = 0; k < body->triangleCount; ++k )
	{
		int ia = body->triangles[3 * k + 0];
		int ib = body->triangles[3 * k + 1];
		int ic = body->triangles[3 * k + 2];
		b3Vec3 a = b3Sub( p[ia], c );
		b3Vec3 b = b3Sub( p[ib], c );
		b3Vec3 cc = b3Sub( p[ic], c );
		volume += b3Dot( a, b3Cross( b, cc ) );
		grad[ia] = b3Add( grad[ia], b3Cross( b, cc ) );
		grad[ib] = b3Add( grad[ib], b3Cross( cc, a ) );
		grad[ic] = b3Add( grad[ic], b3Cross( a, b ) );
	}
	volume /= 6.0f;

	float wGrad2 = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		grad[i] = b3MulSV( 1.0f / 6.0f, grad[i] );
		wGrad2 += w[i] * b3LengthSquared( grad[i] );
	}
	if ( wGrad2 < 1.0e-9f )
	{
		return;
	}

	float c0 = volume - body->pressure * body->restVolume;
	float alpha = b3SoftAlpha( body->volumeSoftness, wGrad2 );
	float dLambda = ( -c0 - alpha * body->volumeLambda ) / ( wGrad2 + alpha );
	body->volumeLambda += dLambda;

	// centroid preserving projection: subtract the mean displacement, then weight by inverse
	// mass so pinned particles stay put
	b3Vec3 mean = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		mean = b3MulAdd( mean, dLambda, grad[i] );
	}
	mean = b3MulSV( 1.0f / (float)n, mean );

	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 corr = b3Sub( b3MulSV( dLambda, grad[i] ), mean );
		p[i] = b3MulAdd( p[i], w[i], corr );
	}
}

// Per-tetrahedron volume constraint: makes a tet-filled body a true solid.
static void b3SoftBodySolveTetVolumes( b3SoftBody* body )
{
	b3Vec3* p = body->p;
	const float* w = body->invMass;
	float softness = body->volumeSoftness;

	for ( int t = 0; t < body->tetCount; ++t )
	{
		b3SoftTet* tet = body->tets + t;
		int i0 = tet->i[0], i1 = tet->i[1], i2 = tet->i[2], i3 = tet->i[3];
		b3Vec3 p0 = p[i0], p1 = p[i1], p2 = p[i2], p3 = p[i3];

		// dV/dp_i for V = dot(p1-p0, cross(p2-p0, p3-p0)) / 6
		b3Vec3 g1 = b3MulSV( 1.0f / 6.0f, b3Cross( b3Sub( p2, p0 ), b3Sub( p3, p0 ) ) );
		b3Vec3 g2 = b3MulSV( 1.0f / 6.0f, b3Cross( b3Sub( p3, p0 ), b3Sub( p1, p0 ) ) );
		b3Vec3 g3 = b3MulSV( 1.0f / 6.0f, b3Cross( b3Sub( p1, p0 ), b3Sub( p2, p0 ) ) );
		b3Vec3 g0 = b3Neg( b3Add( g1, b3Add( g2, g3 ) ) );

		float wsum = w[i0] * b3LengthSquared( g0 ) + w[i1] * b3LengthSquared( g1 ) + w[i2] * b3LengthSquared( g2 ) +
					 w[i3] * b3LengthSquared( g3 );
		if ( wsum < 1.0e-9f )
		{
			continue;
		}

		float volume = b3Dot( b3Sub( p1, p0 ), b3Cross( b3Sub( p2, p0 ), b3Sub( p3, p0 ) ) ) / 6.0f;
		float c = volume - tet->restVolume;
		float alpha = b3SoftAlpha( softness, wsum );
		float dLambda = ( -c - alpha * tet->lambdaVolume ) / ( wsum + alpha );
		tet->lambdaVolume += dLambda;

		p[i0] = b3MulAdd( p[i0], w[i0] * dLambda, g0 );
		p[i1] = b3MulAdd( p[i1], w[i1] * dLambda, g1 );
		p[i2] = b3MulAdd( p[i2], w[i2] * dLambda, g2 );
		p[i3] = b3MulAdd( p[i3], w[i3] * dLambda, g3 );
	}
}

// One XPBD projection of a tet sub-constraint. The vertex gradients are assembled from
// dC/dF (columns pf0..pf2) via G = (dC/dF) * Dm^-T.
static inline float b3ApplyTetConstraint( b3Vec3* p, const b3SoftTet* tet, const float* w, b3Vec3 pf0, b3Vec3 pf1, b3Vec3 pf2,
										  float c, float alpha, float lambda )
{
	b3Vec3 m0 = tet->dmInvC0, m1 = tet->dmInvC1, m2 = tet->dmInvC2;

	b3Vec3 g1 = b3Add( b3MulSV( m0.x, pf0 ), b3Add( b3MulSV( m1.x, pf1 ), b3MulSV( m2.x, pf2 ) ) );
	b3Vec3 g2 = b3Add( b3MulSV( m0.y, pf0 ), b3Add( b3MulSV( m1.y, pf1 ), b3MulSV( m2.y, pf2 ) ) );
	b3Vec3 g3 = b3Add( b3MulSV( m0.z, pf0 ), b3Add( b3MulSV( m1.z, pf1 ), b3MulSV( m2.z, pf2 ) ) );
	b3Vec3 g0 = b3Neg( b3Add( g1, b3Add( g2, g3 ) ) );

	int i0 = tet->i[0], i1 = tet->i[1], i2 = tet->i[2], i3 = tet->i[3];
	float denom = w[i0] * b3LengthSquared( g0 ) + w[i1] * b3LengthSquared( g1 ) + w[i2] * b3LengthSquared( g2 ) +
				  w[i3] * b3LengthSquared( g3 ) + alpha;
	if ( denom < 1.0e-12f )
	{
		return lambda;
	}

	float dLambda = ( -c - alpha * lambda ) / denom;

	p[i0] = b3MulAdd( p[i0], w[i0] * dLambda, g0 );
	p[i1] = b3MulAdd( p[i1], w[i1] * dLambda, g1 );
	p[i2] = b3MulAdd( p[i2], w[i2] * dLambda, g2 );
	p[i3] = b3MulAdd( p[i3], w[i3] * dLambda, g3 );

	return lambda + dLambda;
}

// Stable Neo-Hookean FEM (Macklin & Mueller 2021): per tet a deviatoric (shear) and a
// hydrostatic (volume) sub-constraint on the deformation gradient F = Ds * Dm^-1.
// Compliance is volume weighted (alpha = 1 / (modulus * restVol * dt^2)) so stiffness is
// independent of tessellation. The hydrostatic target gamma = 1 + mu/lambda keeps the
// body from creeping at rest.
static void b3SoftBodySolveNeoHookean( b3SoftBody* body, float sdt )
{
	b3Vec3* p = body->p;
	const float* w = body->invMass;
	float dt2 = sdt * sdt;

	float e = body->youngModulus;
	float nu = body->poissonRatio;
	float mu = e / ( 2.0f * ( 1.0f + nu ) );
	float lame = e * nu / ( ( 1.0f + nu ) * ( 1.0f - 2.0f * nu ) );
	lame = b3MaxFloat( lame, 1.0e-6f );
	float gamma = 1.0f + mu / lame;

	for ( int t = 0; t < body->tetCount; ++t )
	{
		b3SoftTet* tet = body->tets + t;
		int i0 = tet->i[0], i1 = tet->i[1], i2 = tet->i[2], i3 = tet->i[3];
		b3Vec3 m0 = tet->dmInvC0, m1 = tet->dmInvC1, m2 = tet->dmInvC2;

		float volume = b3MaxFloat( b3AbsFloat( tet->restVolume ), 1.0e-9f );
		float alphaDev = 1.0f / ( mu * volume * dt2 );
		float alphaHyd = 1.0f / ( lame * volume * dt2 );

		// deviatoric: C = sqrt(tr(F^T F)), dC/dF = F / C
		{
			b3Vec3 ds0 = b3Sub( p[i1], p[i0] );
			b3Vec3 ds1 = b3Sub( p[i2], p[i0] );
			b3Vec3 ds2 = b3Sub( p[i3], p[i0] );
			b3Vec3 f0 = b3Add( b3MulSV( m0.x, ds0 ), b3Add( b3MulSV( m0.y, ds1 ), b3MulSV( m0.z, ds2 ) ) );
			b3Vec3 f1 = b3Add( b3MulSV( m1.x, ds0 ), b3Add( b3MulSV( m1.y, ds1 ), b3MulSV( m1.z, ds2 ) ) );
			b3Vec3 f2 = b3Add( b3MulSV( m2.x, ds0 ), b3Add( b3MulSV( m2.y, ds1 ), b3MulSV( m2.z, ds2 ) ) );

			float c = sqrtf( b3LengthSquared( f0 ) + b3LengthSquared( f1 ) + b3LengthSquared( f2 ) );
			if ( c > 1.0e-9f )
			{
				float inv = 1.0f / c;
				tet->lambdaDev = b3ApplyTetConstraint( p, tet, w, b3MulSV( inv, f0 ), b3MulSV( inv, f1 ), b3MulSV( inv, f2 ), c,
													   alphaDev, tet->lambdaDev );
			}
		}

		// hydrostatic: C = det(F) - gamma, dC/dF columns are the cofactors of F.
		// Recompute F from the deviatoric-updated positions (Gauss-Seidel within the tet).
		{
			b3Vec3 ds0 = b3Sub( p[i1], p[i0] );
			b3Vec3 ds1 = b3Sub( p[i2], p[i0] );
			b3Vec3 ds2 = b3Sub( p[i3], p[i0] );
			b3Vec3 f0 = b3Add( b3MulSV( m0.x, ds0 ), b3Add( b3MulSV( m0.y, ds1 ), b3MulSV( m0.z, ds2 ) ) );
			b3Vec3 f1 = b3Add( b3MulSV( m1.x, ds0 ), b3Add( b3MulSV( m1.y, ds1 ), b3MulSV( m1.z, ds2 ) ) );
			b3Vec3 f2 = b3Add( b3MulSV( m2.x, ds0 ), b3Add( b3MulSV( m2.y, ds1 ), b3MulSV( m2.z, ds2 ) ) );

			float det = b3Dot( f0, b3Cross( f1, f2 ) );
			tet->lambdaHyd = b3ApplyTetConstraint( p, tet, w, b3Cross( f1, f2 ), b3Cross( f2, f0 ), b3Cross( f0, f1 ),
												   det - gamma, alphaHyd, tet->lambdaHyd );
		}
	}
}

// Shape matching (Mueller 2005): pull particles toward the best-fit rigid placement (plus
// uniform scale) of the rest cloud. Scale-free so the pressure constraint can still inflate.
// Runs once per substep after the iteration loop. Intentionally not mirrored into p0 so the
// pull becomes restoring velocity: that is the wobble.
static void b3SoftBodySolveShapeMatching( b3SoftBody* body )
{
	int n = body->particleCount;
	b3Vec3* p = body->p;
	const float* w = body->invMass;
	const b3Vec3* q = body->rest;

	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		c = b3Add( c, p[i] );
	}
	c = b3MulSV( 1.0f / (float)n, c );

	// covariance Apq = sum (p - c) outer q, held as three columns
	b3Vec3 a0 = b3Vec3_zero, a1 = b3Vec3_zero, a2 = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 d = b3Sub( p[i], c );
		a0 = b3MulAdd( a0, q[i].x, d );
		a1 = b3MulAdd( a1, q[i].y, d );
		a2 = b3MulAdd( a2, q[i].z, d );
	}

	// extract the rotation (Mueller 2016 robust extraction), warm-started across substeps
	b3Quat rot = body->shapeRotation;
	for ( int iter = 0; iter < 20; ++iter )
	{
		b3Vec3 r0 = b3RotateVector( rot, ( b3Vec3 ){ 1.0f, 0.0f, 0.0f } );
		b3Vec3 r1 = b3RotateVector( rot, ( b3Vec3 ){ 0.0f, 1.0f, 0.0f } );
		b3Vec3 r2 = b3RotateVector( rot, ( b3Vec3 ){ 0.0f, 0.0f, 1.0f } );

		float denom = b3AbsFloat( b3Dot( r0, a0 ) + b3Dot( r1, a1 ) + b3Dot( r2, a2 ) ) + 1.0e-9f;
		b3Vec3 omega = b3MulSV( 1.0f / denom, b3Add( b3Cross( r0, a0 ), b3Add( b3Cross( r1, a1 ), b3Cross( r2, a2 ) ) ) );

		float angle = b3Length( omega );
		if ( angle < 1.0e-9f )
		{
			break;
		}

		b3Quat dq = b3MakeQuatFromAxisAngle( b3MulSV( 1.0f / angle, omega ), angle );
		rot = b3NormalizeQuat( b3MulQuat( dq, rot ) );
	}
	body->shapeRotation = rot;

	// best-fit uniform scale of the rotated rest cloud onto the live cloud
	float snum = 0.0f, sden = 0.0f;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 rq = b3RotateVector( rot, q[i] );
		snum += b3Dot( b3Sub( p[i], c ), rq );
		sden += b3LengthSquared( rq );
	}
	float scale = sden > 1.0e-9f ? b3ClampFloat( snum / sden, 0.25f, 4.0f ) : 1.0f;

	float stiffness = body->shapeStiffness;
	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 goal = b3MulAdd( c, scale, b3RotateVector( rot, q[i] ) );
		float k = w[i] > 0.0f ? stiffness : 0.0f;
		p[i] = b3MulAdd( p[i], k, b3Sub( goal, p[i] ) );
	}
}

static void b3SoftBodySolveIteration( b3SoftBody* body, float sdt )
{
	if ( b3SoftBodyEdgesActive( body ) )
	{
		b3SoftBodySolveEdges( body );
	}

	b3SoftBodySolveEnclosedVolume( body );

	if ( body->tetCount > 0 )
	{
		if ( body->enableFem )
		{
			b3SoftBodySolveNeoHookean( body, sdt );
		}
		else
		{
			b3SoftBodySolveTetVolumes( body );
		}
	}
}

// Once per substep after the iteration loop so its strength is decoupled from the
// iteration count.
static void b3SoftBodyPostIteration( b3SoftBody* body )
{
	if ( body->enableShapeMatching )
	{
		b3SoftBodySolveShapeMatching( body );
	}
}

// ---------------------------------------------------------------------------
// World collision response
// ---------------------------------------------------------------------------

// Bounded two-way coupling: a 1-D inelastic contact impulse along the normal using the
// closing velocity and the two masses. It can never inject more energy than the approach
// carries and is self-limiting (once the approach is killed, closing <= 0).
static void b3SoftBodyCoupleRigid( b3World* world, b3SoftBody* body, int shapeId, b3Vec3 hitPointLocal, b3Vec3 normal,
								   b3Vec3 particleVelocity )
{
	b3Shape* shape = b3Array_Get( world->shapes, shapeId );
	b3Body* rigidBody = b3Array_Get( world->bodies, shape->bodyId );
	if ( rigidBody->type != b3_dynamicBody )
	{
		return;
	}

	b3BodyState* state = b3GetBodyState( world, rigidBody );
	if ( state == NULL )
	{
		// sleeping; todo consider waking the body here
		return;
	}

	b3BodySim* sim = b3GetBodySim( world, rigidBody );
	b3Pos hitWorld = b3OffsetPos( body->origin, hitPointLocal );
	b3Vec3 r = b3SubPos( hitWorld, sim->center );

	b3Vec3 rigidVelocity = b3Add( state->linearVelocity, b3Cross( state->angularVelocity, r ) );
	float closing = b3Dot( b3Sub( rigidVelocity, particleVelocity ), normal );
	if ( closing <= 0.0f )
	{
		return;
	}

	closing = b3MinFloat( closing, body->maxParticleSpeed );
	float rigidMass = sim->invMass > 0.0f ? 1.0f / sim->invMass : 0.0f;
	if ( rigidMass <= 0.0f )
	{
		return;
	}

	float softMass = body->particleMass;
	float jMag = closing * ( softMass * rigidMass ) / ( softMass + rigidMass );
	b3Vec3 impulse = b3MulSV( -jMag, normal );

	state->linearVelocity = b3MulAdd( state->linearVelocity, sim->invMass, impulse );
	state->angularVelocity = b3Add( state->angularVelocity, b3MulMV( sim->invInertiaWorld, b3Cross( r, impulse ) ) );
}

// Per-substep swept response: every particle that swept into a surface is snapped to the
// surface and its inward motion removed, with friction on the tangential part. Runs against
// the particle's actual substep motion so it cannot miss fast or edge entries.
static void b3SoftBodyWorldCollide( b3World* world, b3SoftBody* body, float sdt )
{
	B3_UNUSED( sdt );

	if ( body->worldNear == false )
	{
		return;
	}

	int count = body->particleCount;
	float radius = body->particleRadius;

	b3Vec3 bc = b3Vec3_zero;
	for ( int i = 0; i < count; ++i )
	{
		bc = b3Add( bc, body->p[i] );
	}
	bc = b3MulSV( 1.0f / (float)count, bc );

	for ( int i = 0; i < count; ++i )
	{
		if ( body->forceAllTrace == false && b3GetBit( &body->traceCandidates, i ) == false )
		{
			body->safe[i] = body->p[i];
			continue;
		}

		b3Vec3 sweep = b3Sub( body->p[i], body->p0[i] );
		b3SoftCastContext cast = b3SoftBodySphereCast( world, body, body->p0[i], sweep, radius, false );

		b3Vec3 n;
		b3Vec3 hitPoint;
		int hitShapeId;
		float corrAlongN;

		if ( cast.hit && cast.fraction == 0.0f )
		{
			// Started genuinely inside something (a fraction-zero hit carries no valid normal, so
			// every recovery below must produce the contact frame from a cast that starts clear).
			b3Shape* shape = b3Array_Get( world->shapes, cast.shapeId );
			b3Body* rigidBody = b3Array_Get( world->bodies, shape->bodyId );

			if ( rigidBody->type == b3_dynamicBody )
			{
				// A dynamic body moved into this near-stationary particle. Depenetrate out of the
				// intruder along its own outward direction (intruder center to particle), not
				// radially out of the soft body, which would let contact particles wrap around it.
				b3BodySim* sim = b3GetBodySim( world, rigidBody );
				b3Vec3 outDir = b3SubPos( b3OffsetPos( body->origin, body->p[i] ), sim->center );
				if ( b3LengthSquared( outDir ) < 1.0e-8f )
				{
					outDir = b3Sub( body->p[i], bc );
				}
				float outLen = b3Length( outDir );
				outDir = outLen > 1.0e-4f ? b3MulSV( 1.0f / outLen, outDir ) : ( b3Vec3 ){ 0.0f, 0.0f, 1.0f };

				b3Vec3 from = b3MulAdd( body->p[i], body->boundRadius + 2.0f * radius, outDir );
				b3Vec3 back = b3Sub( body->p[i], from );
				b3SoftCastContext rec = b3SoftBodySphereCast( world, body, from, back, radius, false );
				if ( rec.hit == false || rec.fraction == 0.0f )
				{
					continue;
				}

				b3Vec3 before = body->p[i];
				body->p[i] = b3MulAdd( from, rec.fraction, back );
				n = rec.normal;
				hitPoint = rec.point;
				hitShapeId = rec.shapeId;
				corrAlongN = b3Dot( b3Sub( body->p[i], before ), n );
			}
			else
			{
				// Slipped inside static geometry (the thin plane case): the sweep cannot see the
				// surface. Recover with last-safe-position CCD: sweep from the last collision-free
				// anchor; the first surface crossed is the side to return to.
				b3Vec3 recSweep = b3Sub( body->p[i], body->safe[i] );
				b3SoftCastContext rec = b3SoftBodySphereCast( world, body, body->safe[i], recSweep, radius, false );
				if ( rec.hit == false || rec.fraction == 0.0f )
				{
					// no surface between the anchor and here (or the anchor itself is compromised):
					// retreat to the anchor
					body->p[i] = body->safe[i];
					body->p0[i] = body->safe[i];
					continue;
				}

				b3Vec3 before = body->p[i];
				body->p[i] = b3MulAdd( body->safe[i], rec.fraction, recSweep );
				n = rec.normal;
				hitPoint = rec.point;
				hitShapeId = rec.shapeId;
				corrAlongN = b3Dot( b3Sub( body->p[i], before ), n );
			}
		}
		else if ( cast.hit )
		{
			b3Vec3 before = body->p[i];
			body->p[i] = b3MulAdd( body->p0[i], cast.fraction, sweep );
			n = cast.normal;
			hitPoint = cast.point;
			hitShapeId = cast.shapeId;
			float into = b3Dot( b3Sub( before, body->p0[i] ), n );
			corrAlongN = into < 0.0f ? -into : 0.0f;
		}
		else
		{
			// fully clear this substep; this position becomes the collision-free anchor
			body->safe[i] = body->p[i];
			continue;
		}

		if ( corrAlongN > 0.0f )
		{
			b3SoftBodyCoupleRigid( world, body, hitShapeId, hitPoint, n, body->velocity[i] );
		}

		b3Vec3 delta = b3Sub( body->p[i], body->p0[i] );
		float dn = b3Dot( delta, n );
		if ( dn < 0.0f )
		{
			delta = b3MulSub( delta, dn, n );
		}
		b3Vec3 tangent = b3MulSub( delta, b3Dot( delta, n ), n );
		delta = b3MulSub( delta, body->friction, tangent );
		body->p0[i] = b3Sub( body->p[i], delta );

		// resolved on the correct side; record it (nudged just outside) as the new anchor
		body->safe[i] = b3MulAdd( body->p[i], B3_SOFT_SAFE_MARGIN, n );
	}
}

// Safety net: cap how far any particle can stray from the body centroid relative to its
// rest distance, so a snagged vertex cannot stretch into a spike. Moves p0 with the
// correction: a pure relocation that injects no velocity.
static void b3SoftBodyContain( b3SoftBody* body )
{
	if ( body->maxStretch <= 0.0f )
	{
		return;
	}

	int n = body->particleCount;
	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		c = b3Add( c, body->p[i] );
	}
	c = b3MulSV( 1.0f / (float)n, c );

	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 d = b3Sub( body->p[i], c );
		float len = b3Length( d );
		float maxLen = body->restRadius[i] * body->maxStretch;
		if ( len > maxLen && len > 1.0e-5f )
		{
			b3Vec3 corr = b3MulSV( maxLen / len - 1.0f, d );
			body->p[i] = b3Add( body->p[i], corr );
			body->p0[i] = b3Add( body->p0[i], corr );
		}
	}
}

static void b3SoftBodyDeriveVelocities( b3SoftBody* body, float invSdt )
{
	int n = body->particleCount;
	float maxSpeed = body->maxParticleSpeed;

	for ( int i = 0; i < n; ++i )
	{
		b3Vec3 v = b3MulSV( invSdt, b3Sub( body->p[i], body->p0[i] ) );

		// One exploded particle would otherwise leak NaN into the hash and casts and stall
		// the whole step. Park it at its previous position, at rest.
		if ( b3IsValidVec3( v ) == false )
		{
			body->p[i] = body->p0[i];
			body->velocity[i] = b3Vec3_zero;
			continue;
		}

		float speed = b3Length( v );
		if ( speed > maxSpeed )
		{
			v = b3MulSV( maxSpeed / speed, v );
		}
		body->velocity[i] = v;
	}
}

// Re-center the local frame on the particle centroid so particle floats stay small even in
// large worlds. Pure frame shift: world positions are unchanged.
static void b3SoftBodyRecenter( b3SoftBody* body )
{
	int n = body->particleCount;
	b3Vec3 c = b3Vec3_zero;
	for ( int i = 0; i < n; ++i )
	{
		c = b3Add( c, body->p[i] );
	}
	c = b3MulSV( 1.0f / (float)n, c );

	for ( int i = 0; i < n; ++i )
	{
		body->p[i] = b3Sub( body->p[i], c );
		body->p0[i] = b3Sub( body->p0[i], c );
		body->safe[i] = b3Sub( body->safe[i], c );
	}

	body->origin = b3OffsetPos( body->origin, c );
}

// ---------------------------------------------------------------------------
// Self and inter-body particle collision: flat spatial hash + volume depenetration
// ---------------------------------------------------------------------------

static inline int b3SoftHashFloor( float v, float invCell )
{
	return (int)floorf( v * invCell );
}

// Mask each field to 21 bits so an out-of-bounds cell coordinate cannot sign-extend into
// the neighbouring fields, which would collapse stray particles into one bucket.
static inline uint64_t b3SoftHashPack( int x, int y, int z )
{
	return ( ( (uint64_t)( x + 100000 ) ) & 0x1FFFFF ) | ( ( ( (uint64_t)( y + 100000 ) ) & 0x1FFFFF ) << 21 ) |
		   ( ( ( (uint64_t)( z + 100000 ) ) & 0x1FFFFF ) << 42 );
}

static inline int b3SoftHashSlot( uint64_t key, int tableCapacity )
{
	// 64 bit finalizer (splitmix), table capacity is a power of two
	key ^= key >> 33;
	key *= 0xff51afd7ed558ccdull;
	key ^= key >> 33;
	return (int)( key & (uint64_t)( tableCapacity - 1 ) );
}

typedef struct b3SoftCollideSet
{
	b3SoftBody** bodies;
	int bodyCount;

	// gathered particle arrays, one entry per particle of every active body
	b3Vec3* positions; // in the shared reference frame
	const b3Vec3** rest;
	int* bodyIndex;
	int* particleStart; // per body offset into the gathered arrays

	// hash table
	uint64_t* keys;
	int* head;
	int* next;
	int tableCapacity;

	int total;
	float cellSize;
} b3SoftCollideSet;

static void b3SoftBodyResolveParticleCollisions( b3SoftCollideSet* set, int collisionIterations )
{
	int total = set->total;
	float cell = set->cellSize;
	float invCell = 1.0f / cell;

	// build the hash once per substep; pushes are sub-cell so buckets stay valid enough
	memset( set->head, 0xff, set->tableCapacity * sizeof( int ) );
	for ( int i = 0; i < total; ++i )
	{
		b3Vec3 gp = set->positions[i];
		uint64_t key = b3SoftHashPack( b3SoftHashFloor( gp.x, invCell ), b3SoftHashFloor( gp.y, invCell ),
									   b3SoftHashFloor( gp.z, invCell ) );
		int slot = b3SoftHashSlot( key, set->tableCapacity );
		set->keys[i] = key;
		set->next[i] = set->head[slot];
		set->head[slot] = i;
	}

	for ( int ci = 0; ci < collisionIterations; ++ci )
	{
		for ( int i = 0; i < total; ++i )
		{
			b3SoftBody* bodyI = set->bodies[set->bodyIndex[i]];
			float radiusI = bodyI->collisionRadius;
			bool selfI = bodyI->enableSelfCollision;
			bool interI = bodyI->enableInterBodyCollision;

			int cx = b3SoftHashFloor( set->positions[i].x, invCell );
			int cy = b3SoftHashFloor( set->positions[i].y, invCell );
			int cz = b3SoftHashFloor( set->positions[i].z, invCell );

			for ( int dx = -1; dx <= 1; ++dx )
			{
				for ( int dy = -1; dy <= 1; ++dy )
				{
					for ( int dz = -1; dz <= 1; ++dz )
					{
						uint64_t key = b3SoftHashPack( cx + dx, cy + dy, cz + dz );
						int slot = b3SoftHashSlot( key, set->tableCapacity );
						for ( int j = set->head[slot]; j >= 0; j = set->next[j] )
						{
							if ( j <= i || set->keys[j] != key )
							{
								continue;
							}

							b3SoftBody* bodyJ = set->bodies[set->bodyIndex[j]];

							// per pair iteration budget = max of the two bodies' iteration counts
							int pairIterations = b3MaxInt( bodyI->collisionIterations, bodyJ->collisionIterations );
							if ( ci >= pairIterations )
							{
								continue;
							}

							float minDist = radiusI + bodyJ->collisionRadius;

							if ( set->bodyIndex[i] == set->bodyIndex[j] )
							{
								// self collision: enabled and only true folds (far apart in rest shape)
								if ( selfI == false )
								{
									continue;
								}
								int base = set->particleStart[set->bodyIndex[i]];
								b3Vec3 restDelta = b3Sub( set->rest[set->bodyIndex[i]][i - base],
														  set->rest[set->bodyIndex[i]][j - base] );
								float excl = minDist * bodyI->selfExclusionScale;
								if ( b3LengthSquared( restDelta ) < excl * excl )
								{
									continue;
								}
							}
							else
							{
								// inter body: both bodies must opt in
								if ( interI == false || bodyJ->enableInterBodyCollision == false )
								{
									continue;
								}
							}

							b3Vec3 d = b3Sub( set->positions[i], set->positions[j] );
							float dist = b3Length( d );
							if ( dist > 1.0e-5f && dist < minDist )
							{
								float push = 0.5f * ( minDist - dist );
								b3Vec3 nrm = b3MulSV( 1.0f / dist, d );
								set->positions[i] = b3MulAdd( set->positions[i], push, nrm );
								set->positions[j] = b3MulSub( set->positions[j], push, nrm );
							}
						}
					}
				}
			}
		}
	}

	// Volume depenetration: the hash only sees nearby surface particles; a particle that
	// punched through a thin shell into another body's hollow interior sits far from any of
	// that body's surface particles. Push any inter-body particle found inside another
	// body's (centroid, mean radius) sphere back out, clamped to one cell per substep.
	for ( int k = 0; k < set->bodyCount; ++k )
	{
		b3SoftBody* body = set->bodies[k];
		if ( body->enableInterBodyCollision == false )
		{
			continue;
		}

		int start = set->particleStart[k];
		int n = body->particleCount;

		b3Vec3 center = b3Vec3_zero;
		for ( int i = 0; i < n; ++i )
		{
			center = b3Add( center, set->positions[start + i] );
		}
		center = b3MulSV( 1.0f / (float)n, center );

		float meanRadius = 0.0f;
		for ( int i = 0; i < n; ++i )
		{
			meanRadius += b3Distance( set->positions[start + i], center );
		}
		meanRadius /= (float)n;

		for ( int i = 0; i < total; ++i )
		{
			if ( set->bodyIndex[i] == k )
			{
				continue;
			}
			b3SoftBody* other = set->bodies[set->bodyIndex[i]];
			if ( other->enableInterBodyCollision == false )
			{
				continue;
			}

			b3Vec3 dc = b3Sub( set->positions[i], center );
			float dlen = b3Length( dc );
			if ( dlen < meanRadius && dlen > 1.0e-4f )
			{
				float push = b3MinFloat( 0.5f * ( meanRadius - dlen ), cell );
				set->positions[i] = b3MulAdd( set->positions[i], push / dlen, dc );
			}
		}
	}
}

// ---------------------------------------------------------------------------
// Main step
// ---------------------------------------------------------------------------

void b3SolveSoftBodies( b3World* world, float dt )
{
	// gather live bodies
	int capacity = world->softBodies.count;
	int bodyCount = 0;
	for ( int i = 0; i < capacity; ++i )
	{
		if ( world->softBodies.data[i].softBodyId == i && world->softBodies.data[i].particleCount > 0 )
		{
			bodyCount += 1;
		}
	}

	if ( bodyCount == 0 )
	{
		return;
	}

	b3Stack* stack = &world->stack;
	b3SoftBody** bodies = (b3SoftBody**)b3StackAlloc( stack, bodyCount * sizeof( b3SoftBody* ), "soft bodies" );
	{
		int n = 0;
		for ( int i = 0; i < capacity; ++i )
		{
			b3SoftBody* body = world->softBodies.data + i;
			if ( body->softBodyId == i && body->particleCount > 0 )
			{
				bodies[n++] = body;
			}
		}
	}

	// reconcile per-body controls by max across bodies (one shared loop)
	int substeps = 1;
	int iterations = 1;
	int collisionIterations = 1;
	for ( int k = 0; k < bodyCount; ++k )
	{
		substeps = b3MaxInt( substeps, bodies[k]->substepCount );
		iterations = b3MaxInt( iterations, bodies[k]->iterations );
		collisionIterations = b3MaxInt( collisionIterations, bodies[k]->collisionIterations );
	}

	float sdt = dt / (float)substeps;
	float invSdt = 1.0f / sdt;
	b3Vec3 gravity = world->gravity;

	// once per step: world collision culling
	for ( int k = 0; k < bodyCount; ++k )
	{
		b3SoftBodyPrepareWorldCollision( world, bodies[k], dt );
	}

	// broadphase for self/inter particle collision: which bodies enter the hash
	b3SoftCollideSet collideSet = { 0 };
	b3SoftBody** activeBodies = (b3SoftBody**)b3StackAlloc( stack, bodyCount * sizeof( b3SoftBody* ), "soft active" );
	b3Vec3* activeOffsets = NULL;
	{
		int activeCount = 0;
		int interCount = 0;
		bool* isActive = (bool*)b3StackAlloc( stack, bodyCount * sizeof( bool ), "soft active flags" );
		memset( isActive, 0, bodyCount * sizeof( bool ) );

		for ( int k = 0; k < bodyCount; ++k )
		{
			if ( bodies[k]->enableSelfCollision )
			{
				isActive[k] = true;
			}
			if ( bodies[k]->enableInterBodyCollision )
			{
				interCount += 1;
			}
		}

		if ( interCount >= 2 )
		{
			// body-vs-body bounding sphere overlap in world space
			for ( int a = 0; a < bodyCount; ++a )
			{
				if ( bodies[a]->enableInterBodyCollision == false )
				{
					continue;
				}
				for ( int b = a + 1; b < bodyCount; ++b )
				{
					if ( bodies[b]->enableInterBodyCollision == false )
					{
						continue;
					}
					float ra = bodies[a]->boundRadius * 1.6f + bodies[a]->particleRadius;
					float rb = bodies[b]->boundRadius * 1.6f + bodies[b]->particleRadius;
					b3Vec3 d = b3SubPos( bodies[b]->origin, bodies[a]->origin );
					float rr = ra + rb;
					if ( b3LengthSquared( d ) < rr * rr )
					{
						isActive[a] = true;
						isActive[b] = true;
					}
				}
			}
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			if ( isActive[k] )
			{
				activeBodies[activeCount++] = bodies[k];
			}
		}

		b3StackFree( stack, isActive );

		if ( activeCount > 0 )
		{
			int total = 0;
			float cell = 0.0f;
			for ( int k = 0; k < activeCount; ++k )
			{
				total += activeBodies[k]->particleCount;
				cell = b3MaxFloat( cell, 2.0f * activeBodies[k]->collisionRadius );
			}
			cell = b3MaxFloat( cell, 0.001f );

			int tableCapacity = 4;
			while ( tableCapacity < 2 * total )
			{
				tableCapacity *= 2;
			}

			collideSet.bodies = activeBodies;
			collideSet.bodyCount = activeCount;
			collideSet.total = total;
			collideSet.cellSize = cell;
			collideSet.tableCapacity = tableCapacity;
			collideSet.positions = (b3Vec3*)b3StackAlloc( stack, total * sizeof( b3Vec3 ), "soft hash pos" );
			collideSet.rest = (const b3Vec3**)b3StackAlloc( stack, activeCount * sizeof( b3Vec3* ), "soft hash rest" );
			collideSet.bodyIndex = (int*)b3StackAlloc( stack, total * sizeof( int ), "soft hash body" );
			collideSet.particleStart = (int*)b3StackAlloc( stack, activeCount * sizeof( int ), "soft hash start" );
			collideSet.keys = (uint64_t*)b3StackAlloc( stack, total * sizeof( uint64_t ), "soft hash keys" );
			collideSet.head = (int*)b3StackAlloc( stack, tableCapacity * sizeof( int ), "soft hash head" );
			collideSet.next = (int*)b3StackAlloc( stack, total * sizeof( int ), "soft hash next" );

			// particles from different bodies must share one float frame: use the first active
			// body's origin as the reference
			activeOffsets = (b3Vec3*)b3StackAlloc( stack, activeCount * sizeof( b3Vec3 ), "soft hash offsets" );
			int g = 0;
			for ( int k = 0; k < activeCount; ++k )
			{
				activeOffsets[k] = b3SubPos( activeBodies[k]->origin, activeBodies[0]->origin );
				collideSet.rest[k] = activeBodies[k]->rest;
				collideSet.particleStart[k] = g;
				for ( int i = 0; i < activeBodies[k]->particleCount; ++i )
				{
					collideSet.bodyIndex[g++] = k;
				}
			}
		}
	}

	// substep loop
	for ( int s = 0; s < substeps; ++s )
	{
		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyIntegrate( bodies[k], gravity, sdt );
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyBeginSubstep( bodies[k] );
		}

		for ( int it = 0; it < iterations; ++it )
		{
			for ( int k = 0; k < bodyCount; ++k )
			{
				b3SoftBodySolveIteration( bodies[k], sdt );
			}
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyPostIteration( bodies[k] );
		}

		if ( collideSet.bodyCount > 0 )
		{
			// gather into the shared frame, resolve, scatter back
			int g = 0;
			for ( int k = 0; k < collideSet.bodyCount; ++k )
			{
				b3SoftBody* body = collideSet.bodies[k];
				for ( int i = 0; i < body->particleCount; ++i )
				{
					collideSet.positions[g++] = b3Add( body->p[i], activeOffsets[k] );
				}
			}

			b3SoftBodyResolveParticleCollisions( &collideSet, collisionIterations );

			g = 0;
			for ( int k = 0; k < collideSet.bodyCount; ++k )
			{
				b3SoftBody* body = collideSet.bodies[k];
				for ( int i = 0; i < body->particleCount; ++i )
				{
					body->p[i] = b3Sub( collideSet.positions[g++], activeOffsets[k] );
				}
			}
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyWorldCollide( world, bodies[k], sdt );
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyContain( bodies[k] );
		}

		for ( int k = 0; k < bodyCount; ++k )
		{
			b3SoftBodyDeriveVelocities( bodies[k], invSdt );
		}
	}

	for ( int k = 0; k < bodyCount; ++k )
	{
		b3SoftBodyRecenter( bodies[k] );
	}

	// free stack allocations in reverse order
	if ( collideSet.bodyCount > 0 )
	{
		b3StackFree( stack, activeOffsets );
		b3StackFree( stack, collideSet.next );
		b3StackFree( stack, collideSet.head );
		b3StackFree( stack, collideSet.keys );
		b3StackFree( stack, collideSet.particleStart );
		b3StackFree( stack, collideSet.bodyIndex );
		b3StackFree( stack, (void*)collideSet.rest );
		b3StackFree( stack, collideSet.positions );
	}
	b3StackFree( stack, activeBodies );
	b3StackFree( stack, bodies );
}
