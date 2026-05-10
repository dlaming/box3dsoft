// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "utility.h"

#include "box3d/math_functions.h"

struct Camera;
struct b3DebugShape;
struct Scene;
struct UniqueMesh;

enum struct DebugShapeType
{
	capsule,
	compound,
	hull,
	heightField,
	mesh,
	sphere,
};

struct DebugSphere
{
	b3Vec3 center;
	float radius;
};

struct DebugCapsule
{
	b3Vec3 center1, center2;
	float radius;
};

struct DebugHeightField
{
	uint32_t id;
};

struct DebugHull
{
	b3Transform transform;
	uint32_t id;
};

struct DebugMesh
{
	b3Transform transform;
	b3Vec3 scale;
	uint32_t id;
};

struct DebugCompound
{
	DebugCapsule* capsules;
	int capsuleCount;

	DebugHull* hulls;
	int hullCount;

	DebugMesh* meshes;
	int meshCount;

	DebugSphere* spheres;
	int sphereCount;
};

struct DebugShape
{
	DebugShapeType type;
	b3BodyType bodyType;
	union
	{
		DebugCapsule capsule;
		DebugCompound compound;
		DebugHeightField heightField;
		DebugHull hull;
		DebugMesh mesh;
		DebugSphere sphere;
	};
};

Scene* CreateScene( const Camera* camera, Arena arena );
void DestroyScene( Scene* scene );

// Register a hull and get an id to render it later
uint32_t CreateInstanceHull( Scene* scene, const b3Hull* hull, Arena arena );

// Register a mesh and get an id to render it later
uint32_t CreateInstanceMesh( Scene* scene, const b3MeshData* meshData, Arena arena );

// Register a height field and get an id to render it later
uint32_t CreateInstanceHeightField( Scene* scene, const b3HeightField* heightField, Arena arena );

// This should be called when a sample is destroyed to avoid leaking instances
void DestroySharedMeshes( Scene* scene );

void ResizeScene( Scene* scene, const Camera* camera );

// UniqueMesh CreateUniqueMesh( const b3Vec3* vertexPositions, const b3Vec3* vertexNormals, int triangleCount,
//							 const int* triangleIndices, const Edge* edges, int edgeCount, const int* materials,
//							  Arena arena );
// void DestroyUniqueMesh( UniqueMesh* shape );

void DrawDebugShape( Scene* scene, DebugShape* debugShape, b3Transform transform, b3HexColor color, float alpha );

void DrawScene( Scene* scene, Camera* camera );

void DrawCube( Scene* scene, b3Transform transform, b3HexColor color, const Matrix4& view, const Matrix4& projection );

void DrawDebugCube( Scene* scene, b3Transform transform, b3HexColor color );

struct SceneAOSettings
{
	float radius;
	float bias;
	float minScale;
	float power;
	float direct;
	bool aoOnly;
};

// Mutable view of the scene's SSAO parameters. Pointer is valid for the lifetime of the scene.
SceneAOSettings* GetAOSettings( Scene* scene );

void EnableShadows( Scene* scene, bool useShadows );
bool AreShadowsEnabled( Scene* scene );

void EnableSSAO( Scene* scene, bool enable );
bool IsSSAOEnabled( Scene* scene );

// World-space ground grid drawn on near-horizontal upward faces. Useful on
// flat-box ground; should be disabled for samples whose ground is a mesh
// (the mesh's triangle wireframe already provides the grid signal).
void EnableGrid( Scene* scene, bool enable );
bool IsGridEnabled( Scene* scene );

void DrawPoint( Scene* scene, b3Vec3 point, float size, b3HexColor color );
void DrawLine( Scene* scene, b3Vec3 point1, b3Vec3 point2, b3HexColor color );
void DrawPlane( Scene* scene, b3Vec3 normal, b3Vec3 point, b3HexColor color );
void DrawArrow( Scene* scene, b3Vec3 point1, b3Vec3 point2, float size, b3HexColor color );

void DrawBounds( Scene* scene, b3AABB bounds, float extension, b3HexColor color );
void DrawBox( Scene* scene, b3Vec3 extents, b3Transform transform, b3HexColor color );
void DrawFace( Scene* scene, b3Vec3 vertex1, b3Vec3 vertex2, b3Vec3 vertex3, b3HexColor color );
