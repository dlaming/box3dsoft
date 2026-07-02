// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "id.h"
#include "math_functions.h"

#include <stdbool.h>
#include <stdint.h>

/**
 * @defgroup softbody Soft Bodies
 * Particle-based deformable bodies solved with XPBD (extended position based dynamics).
 *
 * A soft body is a cloud of particles connected by constraints:
 * - distance constraints on every unique edge (structural stiffness)
 * - a single enclosed-volume (gas pressure) constraint for hollow shells built from surface triangles
 * - per-tetrahedron volume constraints for solids built from tetrahedra
 * - an optional stable Neo-Hookean FEM model (per-tet deviatoric + hydrostatic) for solids
 * - optional shape matching (global return-to-rest-shape) for shells
 *
 * Soft bodies collide with rigid body shapes using per-substep swept sphere casts and can
 * optionally collide with themselves and each other through a particle spatial hash.
 * Two-way coupling applies bounded contact impulses to dynamic rigid bodies.
 * @{
 */

/// Soft body id references a soft body instance. This should be treated as an opaque handle.
typedef struct b3SoftBodyId
{
	int32_t index1;
	uint16_t world0;
	uint16_t generation;
} b3SoftBodyId;

/// A null soft body id.
static const b3SoftBodyId b3_nullSoftBodyId = B3_NULL_ID;

/// Definition of a soft body. The mesh topology is supplied as raw arrays:
/// - `restPositions` are the particle positions in local space (required)
/// - `triangles` are surface triangles (3 indices each) used for the enclosed-volume constraint,
///   surface edges, and shape matching. Required for hollow shells, optional for solids.
/// - `tets` are tetrahedra (4 indices each) used for per-tet volume / FEM constraints and tet edges.
///   Required for solids.
/// The definition arrays are copied by b3CreateSoftBody() and may be freed afterwards.
/// Must be initialized using b3DefaultSoftBodyDef().
typedef struct b3SoftBodyDef
{
	/// World-space origin. Particles start at origin + restPositions[i].
	b3Pos origin;

	/// Particle rest positions in local space.
	const b3Vec3* restPositions;
	int particleCount;

	/// Surface triangles, 3 particle indices each, outward winding. May be NULL.
	const int* triangles;
	int triangleCount;

	/// Tetrahedra, 4 particle indices each. May be NULL.
	const int* tets;
	int tetCount;

	/// Total mass of the body in kilograms, distributed uniformly across particles. Must be positive.
	float mass;

	/// Substeps per world step. Soft bodies substep independently of the rigid solver.
	int substepCount;

	/// Constraint solver iterations per substep. Usually 1 with many substeps (small steps).
	int iterations;

	/// Edge (distance constraint) softness in [0,1]. 0 = rigid springs (PBD), 1 = very soft.
	/// This maps to an XPBD compliance scaled by the constraint's own gradient magnitude so the
	/// knob is stable across mesh scale and substep count.
	float edgeSoftness;

	/// Volume constraint softness in [0,1]. 0 = incompressible, 1 = springy (gas-like).
	float volumeSoftness;

	/// Volume target scale for shells (gas pressure). 1 = rest volume, >1 inflates.
	float pressure;

	/// Enable shape matching for shells: a global pull back toward the rest shape (jello).
	/// Replaces the structural edge springs while enabled unless keepEdgesWithShapeMatching is set.
	bool enableShapeMatching;

	/// Shape matching stiffness in [0,1]: ~1 = stiff rubber, low = floppy jello.
	float shapeStiffness;

	/// Keep edge springs active alongside shape matching. Recommended for arbitrary meshes which
	/// have no implied structure and can collapse under shape matching alone.
	bool keepEdgesWithShapeMatching;

	/// Use the stable Neo-Hookean FEM model for tets instead of edge springs + per-tet volume.
	/// Rotation-correct and inversion-safe. Tuned by youngModulus/poissonRatio.
	bool enableFem;

	/// Young's modulus (Pa) for the FEM model. Higher = firmer.
	float youngModulus;

	/// Poisson ratio [0, 0.49] for the FEM model. Near 0.5 = incompressible, rubber-like.
	float poissonRatio;

	/// Rigid-mode damping rate (1/s). Bleeds velocity toward the body's rigid motion
	/// (center of mass velocity + rotation), preserving linear and angular momentum, so it
	/// removes internal jiggle without weakening gravity or slowing falls.
	float damping;

	/// Collision radius of each particle for world collision. 0 = derive from mean edge length.
	float particleRadius;

	/// Scale applied to the derived self/inter collision radius (half mean edge length).
	float collisionRadiusScale;

	/// Friction coefficient in [0,1] applied to tangential particle motion at contacts.
	float friction;

	/// Maximum particle speed (m/s). A safety clamp; also sizes the collision culling band.
	float maxParticleSpeed;

	/// Enable collision between particles of this body (folding onto itself).
	bool enableSelfCollision;

	/// Enable collision with other soft bodies that also have this flag.
	bool enableInterBodyCollision;

	/// Self/inter particle collision solve passes per substep.
	int collisionIterations;

	/// Self-collision rest-neighbour exclusion scale. Particles closer than this many collision
	/// radii in the REST shape never self-collide (too low = skin jitter, too high = folds clip).
	float selfExclusionScale;

	/// Cap each particle's distance from the body centroid at (rest distance) x maxStretch.
	/// A cheap safety net that stops a snagged particle stretching into a spike. 0 = disabled.
	float maxStretch;

	/// Collision filter category/mask bits for world collision.
	uint64_t categoryBits;
	uint64_t maskBits;

	/// User data pointer.
	void* userData;

	/// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
} b3SoftBodyDef;

/// Use this to initialize your soft body definition.
B3_API b3SoftBodyDef b3DefaultSoftBodyDef( void );

/// Create a soft body from a definition. The definition arrays are copied.
B3_API b3SoftBodyId b3CreateSoftBody( b3WorldId worldId, const b3SoftBodyDef* def );

/// Destroy a soft body.
B3_API void b3DestroySoftBody( b3SoftBodyId softBodyId );

/// Soft body identifier validation. Can be used to detect orphaned ids.
B3_API bool b3SoftBody_IsValid( b3SoftBodyId id );

/// Get the number of particles.
B3_API int b3SoftBody_GetParticleCount( b3SoftBodyId softBodyId );

/// Copy the current particle positions into `positions` (capacity >= particle count).
/// Use this to skin a render mesh.
B3_API void b3SoftBody_GetParticlePositions( b3SoftBodyId softBodyId, b3Pos* positions, int capacity );

/// Get the current particle centroid (uniform mass).
B3_API b3Pos b3SoftBody_GetCentroid( b3SoftBodyId softBodyId );

/// Teleport the soft body so its centroid is at `position`, preserving the deformed shape.
/// Velocities are zeroed.
B3_API void b3SoftBody_SetPosition( b3SoftBodyId softBodyId, b3Pos position );

/// Apply a linear impulse (N*s) to every particle, distributed by mass.
B3_API void b3SoftBody_ApplyLinearImpulse( b3SoftBodyId softBodyId, b3Vec3 impulse );

/// Pin or release a particle. A pinned particle has zero inverse mass and is unaffected by
/// constraints and gravity; move it with b3SoftBody_SetParticlePosition.
B3_API void b3SoftBody_SetParticlePinned( b3SoftBodyId softBodyId, int particleIndex, bool pinned );

/// Set a particle position directly (world space). Intended for pinned/kinematic particles.
B3_API void b3SoftBody_SetParticlePosition( b3SoftBodyId softBodyId, int particleIndex, b3Pos position );

/// Set the volume target scale (gas pressure) for shells.
B3_API void b3SoftBody_SetPressure( b3SoftBodyId softBodyId, float pressure );

/// Enable/disable shape matching.
B3_API void b3SoftBody_EnableShapeMatching( b3SoftBodyId softBodyId, bool flag );

/// Enable/disable the Neo-Hookean FEM model on a tet body.
B3_API void b3SoftBody_EnableFem( b3SoftBodyId softBodyId, bool flag );

/// Set edge/volume softness in [0,1].
B3_API void b3SoftBody_SetSoftness( b3SoftBodyId softBodyId, float edgeSoftness, float volumeSoftness );

/// Set the FEM material.
B3_API void b3SoftBody_SetMaterial( b3SoftBodyId softBodyId, float youngModulus, float poissonRatio );

/// Set the user data pointer.
B3_API void b3SoftBody_SetUserData( b3SoftBodyId softBodyId, void* userData );

/// Get the user data pointer.
B3_API void* b3SoftBody_GetUserData( b3SoftBodyId softBodyId );

/**@}*/
