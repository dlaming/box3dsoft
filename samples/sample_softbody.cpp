// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "gfx/draw.h"
#include "sample.h"

#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include "imgui.h"

#include <map>
#include <vector>

// Shared soft body mesh builders ------------------------------------------------------------

struct SoftMesh
{
	std::vector<b3Vec3> positions;
	std::vector<int> triangles; // render + shell constraint surface
	std::vector<int> tets;		// solid topology, empty for shells
};

// Tetrahedral box lattice: (resolution+1)^3 grid points, each cell split into 6 Kuhn tets
// sharing the main diagonal. The render surface is the boundary faces.
static SoftMesh MakeJelloBox( float halfExtent, int resolution )
{
	SoftMesh mesh;
	int np = resolution + 1;
	float step = 2.0f * halfExtent / (float)resolution;

	for ( int z = 0; z < np; ++z )
	{
		for ( int y = 0; y < np; ++y )
		{
			for ( int x = 0; x < np; ++x )
			{
				mesh.positions.push_back( { -halfExtent + x * step, -halfExtent + y * step, -halfExtent + z * step } );
			}
		}
	}

	auto index = [np]( int x, int y, int z ) {
		return x + y * np + z * np * np;
	};

	for ( int z = 0; z < resolution; ++z )
	{
		for ( int y = 0; y < resolution; ++y )
		{
			for ( int x = 0; x < resolution; ++x )
			{
				int c000 = index( x, y, z ), c100 = index( x + 1, y, z );
				int c010 = index( x, y + 1, z ), c110 = index( x + 1, y + 1, z );
				int c001 = index( x, y, z + 1 ), c101 = index( x + 1, y, z + 1 );
				int c011 = index( x, y + 1, z + 1 ), c111 = index( x + 1, y + 1, z + 1 );

				int kuhn[6][4] = {
					{ c000, c100, c110, c111 }, { c000, c110, c010, c111 }, { c000, c010, c011, c111 },
					{ c000, c011, c001, c111 }, { c000, c001, c101, c111 }, { c000, c101, c100, c111 },
				};

				for ( int k = 0; k < 6; ++k )
				{
					mesh.tets.push_back( kuhn[k][0] );
					mesh.tets.push_back( kuhn[k][1] );
					mesh.tets.push_back( kuhn[k][2] );
					mesh.tets.push_back( kuhn[k][3] );
				}
			}
		}
	}

	// boundary faces: tet faces that belong to exactly one tet, wound outward
	struct FaceRec
	{
		int count, a, b, c, d;
	};
	std::map<uint64_t, FaceRec> faces;
	auto addFace = [&faces]( int a, int b, int c, int d ) {
		int lo = b3MinInt( a, b3MinInt( b, c ) );
		int hi = b3MaxInt( a, b3MaxInt( b, c ) );
		int mid = a + b + c - lo - hi;
		uint64_t key = (uint64_t)lo | ( (uint64_t)mid << 21 ) | ( (uint64_t)hi << 42 );
		auto it = faces.find( key );
		if ( it != faces.end() )
		{
			it->second.count += 1;
		}
		else
		{
			faces[key] = { 1, a, b, c, d };
		}
	};

	for ( size_t t = 0; t < mesh.tets.size(); t += 4 )
	{
		int a = mesh.tets[t], b = mesh.tets[t + 1], c = mesh.tets[t + 2], d = mesh.tets[t + 3];
		addFace( a, b, c, d );
		addFace( a, b, d, c );
		addFace( a, c, d, b );
		addFace( b, c, d, a );
	}

	for ( auto& kv : faces )
	{
		FaceRec rec = kv.second;
		if ( rec.count != 1 )
		{
			continue;
		}
		int a = rec.a, b = rec.b, c = rec.c;
		b3Vec3 n = b3Cross( b3Sub( mesh.positions[b], mesh.positions[a] ), b3Sub( mesh.positions[c], mesh.positions[a] ) );
		if ( b3Dot( n, b3Sub( mesh.positions[a], mesh.positions[rec.d] ) ) < 0.0f )
		{
			int tmp = b;
			b = c;
			c = tmp;
		}
		mesh.triangles.push_back( a );
		mesh.triangles.push_back( b );
		mesh.triangles.push_back( c );
	}

	return mesh;
}

// Icosphere shell: icosahedron midpoint-subdivided, vertices projected to the sphere.
// Near-uniform valence so deformation is isotropic.
static SoftMesh MakeIcosphere( float radius, int subdivisions )
{
	SoftMesh mesh;

	const float t = 0.5f * ( 1.0f + sqrtf( 5.0f ) );
	b3Vec3 base[12] = {
		{ -1.0f, t, 0.0f }, { 1.0f, t, 0.0f },	 { -1.0f, -t, 0.0f }, { 1.0f, -t, 0.0f },
		{ 0.0f, -1.0f, t }, { 0.0f, 1.0f, t },	 { 0.0f, -1.0f, -t }, { 0.0f, 1.0f, -t },
		{ t, 0.0f, -1.0f }, { t, 0.0f, 1.0f },	 { -t, 0.0f, -1.0f }, { -t, 0.0f, 1.0f },
	};
	for ( int i = 0; i < 12; ++i )
	{
		mesh.positions.push_back( b3MulSV( radius, b3Normalize( base[i] ) ) );
	}

	int faces[20][3] = {
		{ 0, 11, 5 }, { 0, 5, 1 },	{ 0, 1, 7 },   { 0, 7, 10 }, { 0, 10, 11 }, { 1, 5, 9 },  { 5, 11, 4 },
		{ 11, 10, 2 }, { 10, 7, 6 }, { 7, 1, 8 },  { 3, 9, 4 },  { 3, 4, 2 },	 { 3, 2, 6 },  { 3, 6, 8 },
		{ 3, 8, 9 },  { 4, 9, 5 },	{ 2, 4, 11 },  { 6, 2, 10 }, { 8, 6, 7 },	 { 9, 8, 1 },
	};
	for ( int i = 0; i < 20; ++i )
	{
		mesh.triangles.push_back( faces[i][0] );
		mesh.triangles.push_back( faces[i][1] );
		mesh.triangles.push_back( faces[i][2] );
	}

	std::map<uint64_t, int> midpointCache;
	auto midpoint = [&]( int a, int b ) {
		uint64_t key = a < b ? ( (uint64_t)a << 32 ) | (uint32_t)b : ( (uint64_t)b << 32 ) | (uint32_t)a;
		auto it = midpointCache.find( key );
		if ( it != midpointCache.end() )
		{
			return it->second;
		}
		b3Vec3 m = b3MulSV( 0.5f, b3Add( mesh.positions[a], mesh.positions[b] ) );
		m = b3MulSV( radius, b3Normalize( m ) );
		int idx = (int)mesh.positions.size();
		mesh.positions.push_back( m );
		midpointCache[key] = idx;
		return idx;
	};

	for ( int s = 0; s < subdivisions; ++s )
	{
		std::vector<int> next;
		next.reserve( 4 * mesh.triangles.size() );
		for ( size_t k = 0; k < mesh.triangles.size(); k += 3 )
		{
			int a = mesh.triangles[k], b = mesh.triangles[k + 1], c = mesh.triangles[k + 2];
			int ab = midpoint( a, b ), bc = midpoint( b, c ), ca = midpoint( c, a );
			int tris[4][3] = { { a, ab, ca }, { b, bc, ab }, { c, ca, bc }, { ab, bc, ca } };
			for ( int i = 0; i < 4; ++i )
			{
				next.push_back( tris[i][0] );
				next.push_back( tris[i][1] );
				next.push_back( tris[i][2] );
			}
		}
		mesh.triangles.swap( next );
	}

	return mesh;
}

// Draw a soft body's surface triangles (both windings so shells read from inside too),
// positioned relative to the centroid so the draw stays accurate in a large world.
static void DrawSoftBody( b3SoftBodyId id, const SoftMesh& mesh, Vec4 color, bool wireframe )
{
	int count = b3SoftBody_GetParticleCount( id );
	std::vector<b3Pos> points( count );
	b3SoftBody_GetParticlePositions( id, points.data(), count );

	b3Pos centroid = b3SoftBody_GetCentroid( id );
	std::vector<b3Vec3> local( count );
	for ( int i = 0; i < count; ++i )
	{
		local[i] = b3SubPos( points[i], centroid );
	}

	b3WorldTransform transform = { centroid, b3Quat_identity };

	for ( size_t k = 0; k < mesh.triangles.size(); k += 3 )
	{
		b3Vec3 a = local[mesh.triangles[k]];
		b3Vec3 b = local[mesh.triangles[k + 1]];
		b3Vec3 c = local[mesh.triangles[k + 2]];
		DrawTriangle( transform, a, b, c, color );
		DrawTriangle( transform, a, c, b, color );

		if ( wireframe )
		{
			Vec4 lineColor = { 0.1f, 0.1f, 0.1f, 1.0f };
			DrawLine( b3OffsetPos( centroid, a ), b3OffsetPos( centroid, b ), lineColor );
			DrawLine( b3OffsetPos( centroid, b ), b3OffsetPos( centroid, c ), lineColor );
			DrawLine( b3OffsetPos( centroid, c ), b3OffsetPos( centroid, a ), lineColor );
		}
	}
}

// Jello Cube --------------------------------------------------------------------------------

class JelloCube : public Sample
{
public:
	explicit JelloCube( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 2.5f, 2.0f, 2.5f, { 0.0f, 0.5f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		m_mesh = MakeJelloBox( 0.5f, 3 );
		m_useFem = false;
		m_edgeSoftness = 0.1f;
		m_volumeSoftness = 0.0f;
		m_wireframe = false;

		CreateSoftBody();
	}

	void CreateSoftBody()
	{
		b3SoftBodyDef def = b3DefaultSoftBodyDef();
		def.restPositions = m_mesh.positions.data();
		def.particleCount = (int)m_mesh.positions.size();
		def.tets = m_mesh.tets.data();
		def.tetCount = (int)m_mesh.tets.size() / 4;
		def.mass = 5.0f;
		def.origin = { 0.0f, 2.0f, 0.0f };
		def.edgeSoftness = m_edgeSoftness;
		def.volumeSoftness = m_volumeSoftness;
		def.enableFem = m_useFem;
		def.youngModulus = 5.0e4f;
		def.poissonRatio = 0.45f;

		m_softBodyId = b3CreateSoftBody( m_worldId, &def );
	}

	bool DrawControls() override
	{
		bool changed = false;
		if ( ImGui::Checkbox( "FEM (Neo-Hookean)", &m_useFem ) )
		{
			b3SoftBody_EnableFem( m_softBodyId, m_useFem );
			changed = true;
		}

		if ( ImGui::SliderFloat( "Edge Softness", &m_edgeSoftness, 0.0f, 1.0f ) ||
			 ImGui::SliderFloat( "Volume Softness", &m_volumeSoftness, 0.0f, 1.0f ) )
		{
			b3SoftBody_SetSoftness( m_softBodyId, m_edgeSoftness, m_volumeSoftness );
			changed = true;
		}

		if ( ImGui::Button( "Launch" ) )
		{
			b3SoftBody_ApplyLinearImpulse( m_softBodyId, { 0.0f, 25.0f, 0.0f } );
			changed = true;
		}

		ImGui::Checkbox( "Wireframe", &m_wireframe );
		return changed;
	}

	void Render() override
	{
		Sample::Render();
		Vec4 color = { 0.2f, 0.8f, 0.3f, 1.0f };
		DrawSoftBody( m_softBodyId, m_mesh, color, m_wireframe );
	}

	static Sample* Create( SampleContext* context )
	{
		return new JelloCube( context );
	}

	SoftMesh m_mesh;
	b3SoftBodyId m_softBodyId;
	float m_edgeSoftness;
	float m_volumeSoftness;
	bool m_useFem;
	bool m_wireframe;
};

static int sampleJelloCube = RegisterSample( "Soft Body", "Jello Cube", JelloCube::Create );

// Balloon -----------------------------------------------------------------------------------

class Balloon : public Sample
{
public:
	explicit Balloon( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 2.5f, 2.0f, 2.5f, { 0.0f, 0.5f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		m_mesh = MakeIcosphere( 0.5f, 2 );
		m_pressure = 1.2f;
		m_shapeMatching = false;
		m_wireframe = false;

		b3SoftBodyDef def = b3DefaultSoftBodyDef();
		def.restPositions = m_mesh.positions.data();
		def.particleCount = (int)m_mesh.positions.size();
		def.triangles = m_mesh.triangles.data();
		def.triangleCount = (int)m_mesh.triangles.size() / 3;
		def.mass = 1.0f;
		def.origin = { 0.0f, 2.0f, 0.0f };
		def.pressure = m_pressure;
		def.shapeStiffness = 0.2f;

		m_softBodyId = b3CreateSoftBody( m_worldId, &def );
	}

	bool DrawControls() override
	{
		bool changed = false;
		if ( ImGui::SliderFloat( "Pressure", &m_pressure, 0.5f, 3.0f ) )
		{
			b3SoftBody_SetPressure( m_softBodyId, m_pressure );
			changed = true;
		}

		if ( ImGui::Checkbox( "Shape Matching", &m_shapeMatching ) )
		{
			b3SoftBody_EnableShapeMatching( m_softBodyId, m_shapeMatching );
			changed = true;
		}

		if ( ImGui::Button( "Launch" ) )
		{
			b3SoftBody_ApplyLinearImpulse( m_softBodyId, { 3.0f, 5.0f, 0.0f } );
			changed = true;
		}

		ImGui::Checkbox( "Wireframe", &m_wireframe );
		return changed;
	}

	void Render() override
	{
		Sample::Render();
		Vec4 color = { 0.9f, 0.3f, 0.3f, 1.0f };
		DrawSoftBody( m_softBodyId, m_mesh, color, m_wireframe );
	}

	static Sample* Create( SampleContext* context )
	{
		return new Balloon( context );
	}

	SoftMesh m_mesh;
	b3SoftBodyId m_softBodyId;
	float m_pressure;
	bool m_shapeMatching;
	bool m_wireframe;
};

static int sampleBalloon = RegisterSample( "Soft Body", "Balloon", Balloon::Create );

// Soft Pile ---------------------------------------------------------------------------------

// Several soft bodies with inter-body collision, plus rigid boxes for two-way coupling.
class SoftPile : public Sample
{
public:
	static constexpr int m_bodyCount = 4;

	explicit SoftPile( SampleContext* context )
		: Sample( context )
	{
		if ( context->restart == false )
		{
			m_camera->SetView( 4.0f, 3.0f, 4.0f, { 0.0f, 1.0f, 0.0f } );
		}

		AddGroundBox( 10.0f );

		m_mesh = MakeIcosphere( 0.4f, 2 );
		m_wireframe = false;

		for ( int i = 0; i < m_bodyCount; ++i )
		{
			b3SoftBodyDef def = b3DefaultSoftBodyDef();
			def.restPositions = m_mesh.positions.data();
			def.particleCount = (int)m_mesh.positions.size();
			def.triangles = m_mesh.triangles.data();
			def.triangleCount = (int)m_mesh.triangles.size() / 3;
			def.mass = 1.0f;
			def.origin = { 0.15f * ( i - 1 ), 1.0f + 1.2f * i, 0.1f * i };
			def.pressure = 1.1f;
			def.enableInterBodyCollision = true;

			m_softBodyIds[i] = b3CreateSoftBody( m_worldId, &def );
		}

		// rigid boxes dropped on the pile to show two-way coupling
		b3BoxHull box = b3MakeBoxHull( 0.25f, 0.25f, 0.25f );
		b3ShapeDef shapeDef = b3DefaultShapeDef();
		for ( int i = 0; i < 2; ++i )
		{
			b3BodyDef bodyDef = b3DefaultBodyDef();
			bodyDef.type = b3_dynamicBody;
			bodyDef.position = { 0.3f * i - 0.15f, 6.0f + i, 0.0f };
			b3BodyId bodyId = b3CreateBody( m_worldId, &bodyDef );
			b3CreateHullShape( bodyId, &shapeDef, &box.base );
		}
	}

	bool DrawControls() override
	{
		ImGui::Checkbox( "Wireframe", &m_wireframe );
		return false;
	}

	void Render() override
	{
		Sample::Render();
		Vec4 colors[m_bodyCount] = {
			{ 0.9f, 0.4f, 0.2f, 1.0f },
			{ 0.2f, 0.6f, 0.9f, 1.0f },
			{ 0.8f, 0.8f, 0.2f, 1.0f },
			{ 0.6f, 0.3f, 0.8f, 1.0f },
		};
		for ( int i = 0; i < m_bodyCount; ++i )
		{
			DrawSoftBody( m_softBodyIds[i], m_mesh, colors[i], m_wireframe );
		}
	}

	static Sample* Create( SampleContext* context )
	{
		return new SoftPile( context );
	}

	SoftMesh m_mesh;
	b3SoftBodyId m_softBodyIds[m_bodyCount];
	bool m_wireframe;
};

static int sampleSoftPile = RegisterSample( "Soft Body", "Soft Pile", SoftPile::Create );
