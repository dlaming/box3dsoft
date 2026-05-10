// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

// b3GetHeightFieldTriangle is internal
#include "shape.h"

#include "box3d/collision.h"
#include "box3d/math_functions.h"

#include <float.h>
#include <stdio.h>

static int HeightFieldCreate( void )
{
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightField* hf = b3CreateGrid( 4, 4, scale, false );

	ENSURE( hf->rowCount == 4 );
	ENSURE( hf->columnCount == 4 );
	ENSURE( hf->clockwise == false );
	ENSURE( hf->version == B3_HEIGHT_FIELD_VERSION );

	ENSURE_SMALL( hf->aabb.lowerBound.x, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.lowerBound.y, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.lowerBound.z, FLT_EPSILON );

	ENSURE_SMALL( hf->aabb.upperBound.x - 3.0f, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.upperBound.y, FLT_EPSILON );
	ENSURE_SMALL( hf->aabb.upperBound.z - 3.0f, FLT_EPSILON );

	b3DestroyHeightField( hf );
	return 0;
}

static int HeightFieldTriangleIndex( void )
{
	// Asymmetric grid (rows != cols) catches off-by-one errors between
	// vertex stride (columnCount) and cell stride (columnCount - 1).
	int rowCount = 4;
	int columnCount = 5;
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightField* hf = b3CreateGrid( rowCount, columnCount, scale, false );

	int triangleCount = 2 * ( rowCount - 1 ) * ( columnCount - 1 );

	for ( int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex )
	{
		int quadIndex = triangleIndex >> 1;
		int sub = triangleIndex & 1;
		int row = quadIndex / ( columnCount - 1 );
		int column = quadIndex - row * ( columnCount - 1 );

		int index11 = row * columnCount + column;
		int index12 = index11 + 1;
		int index21 = ( row + 1 ) * columnCount + column;
		int index22 = index21 + 1;

		b3Triangle t = b3GetHeightFieldTriangle( hf, triangleIndex );

		if ( sub == 0 )
		{
			// Triangle 0 (CCW): {11, 21, 12}
			ENSURE( t.i1 == index11 );
			ENSURE( t.i2 == index21 );
			ENSURE( t.i3 == index12 );
		}
		else
		{
			// Triangle 1 (CCW): {22, 12, 21}
			ENSURE( t.i1 == index22 );
			ENSURE( t.i2 == index12 );
			ENSURE( t.i3 == index21 );
		}
	}

	b3DestroyHeightField( hf );
	return 0;
}

static int HeightFieldWinding( void )
{
	// Build the same flat 3x3 field with CCW and CW winding. The cross-product
	// normal of triangle 0 must flip sign accordingly.
	float heights[9] = { 0 };
	uint8_t materials[4] = { 0, 0, 0, 0 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;

	def.clockwiseWinding = false;
	b3HeightField* ccw = b3CreateHeightField( &def );

	def.clockwiseWinding = true;
	b3HeightField* cw = b3CreateHeightField( &def );

	b3Triangle ta = b3GetHeightFieldTriangle( ccw, 0 );
	b3Triangle tb = b3GetHeightFieldTriangle( cw, 0 );

	b3Vec3 na =
		b3Normalize( b3Cross( b3Sub( ta.vertices[1], ta.vertices[0] ), b3Sub( ta.vertices[2], ta.vertices[0] ) ) );
	b3Vec3 nb =
		b3Normalize( b3Cross( b3Sub( tb.vertices[1], tb.vertices[0] ), b3Sub( tb.vertices[2], tb.vertices[0] ) ) );

	ENSURE_SMALL( na.x, FLT_EPSILON );
	ENSURE_SMALL( na.y - 1.0f, FLT_EPSILON );
	ENSURE_SMALL( na.z, FLT_EPSILON );

	ENSURE_SMALL( nb.x, FLT_EPSILON );
	ENSURE_SMALL( nb.y + 1.0f, FLT_EPSILON );
	ENSURE_SMALL( nb.z, FLT_EPSILON );

	b3DestroyHeightField( ccw );
	b3DestroyHeightField( cw );
	return 0;
}

static int RayCastFlatField( void )
{
	// Build a flat 4x4 field with a tight quantization range so the recovered
	// surface stays within ~1e-5 of y=0 (b3CreateGrid uses -256..256 which
	// blows the 1/UINT16_MAX quantum up to ~4e-3 in y).
	float heights[16] = { 0 };
	uint8_t materials[9] = { 0 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.0f, 1.0f, 1.0f };
	def.countX = 4;
	def.countZ = 4;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = false;

	b3HeightField* hf = b3CreateHeightField( &def );

	// Origin sits clearly inside triangle 0 of cell (1, 1) — off the cell
	// diagonal x+z = 3. The translation overshoots the surface so the hit
	// fraction is strictly less than maxFraction.
	b3RayCastInput input = { 0 };
	input.origin = (b3Vec3){ 1.25f, 10.0f, 1.25f };
	input.translation = (b3Vec3){ 0.0f, -20.0f, 0.0f };
	input.maxFraction = 1.0f;

	b3CastOutput out = b3RayCastHeightField( hf, &input );

	ENSURE( out.hit == true );
	ENSURE_SMALL( out.fraction - 0.5f, 1e-5f );
	ENSURE_SMALL( out.normal.x, 1e-5f );
	ENSURE_SMALL( out.normal.y - 1.0f, 1e-5f );
	ENSURE_SMALL( out.normal.z, 1e-5f );

	b3DestroyHeightField( hf );
	return 0;
}

static int OverlapAtSurface( void )
{
	b3Vec3 scale = { 1.0f, 1.0f, 1.0f };
	b3HeightField* hf = b3CreateGrid( 4, 4, scale, false );

	// Sphere center 1.0 above the surface, radius 0.5 — clear gap.
	b3Vec3 above = { 1.5f, 1.0f, 1.5f };
	b3ShapeProxy proxyAbove = { &above, 1, 0.5f };
	bool hitAbove = b3OverlapHeightField( hf, b3Transform_identity, &proxyAbove );
	ENSURE( hitAbove == false );

	// Sphere centered on the surface — radius pokes through.
	b3Vec3 through = { 1.5f, 0.0f, 1.5f };
	b3ShapeProxy proxyThrough = { &through, 1, 0.5f };
	bool hitThrough = b3OverlapHeightField( hf, b3Transform_identity, &proxyThrough );
	ENSURE( hitThrough == true );

	b3DestroyHeightField( hf );
	return 0;
}

static int FileRoundtrip( void )
{
	float heights[9] = { 0.0f, 0.5f, -0.3f, 0.1f, 0.0f, 0.0f, 0.0f, 0.2f, 0.0f };
	uint8_t materials[4] = { 0, B3_HEIGHT_FIELD_HOLE, 1, 2 };

	b3HeightFieldDef def = { 0 };
	def.heights = heights;
	def.materialIndices = materials;
	def.scale = (b3Vec3){ 1.5f, 2.0f, 0.75f };
	def.countX = 3;
	def.countZ = 3;
	def.globalMinimumHeight = -1.0f;
	def.globalMaximumHeight = 1.0f;
	def.clockwiseWinding = true;

	const char* path = "test_height_field_roundtrip.dat";
	b3DumpHeightData( &def, path );

	b3HeightField* loaded = b3LoadHeightField( path );
	remove( path );

	ENSURE( loaded != NULL );
	ENSURE( loaded->rowCount == def.countZ );
	ENSURE( loaded->columnCount == def.countX );
	ENSURE( loaded->clockwise == def.clockwiseWinding );

	ENSURE_SMALL( loaded->scale.x - def.scale.x, FLT_EPSILON );
	ENSURE_SMALL( loaded->scale.y - def.scale.y, FLT_EPSILON );
	ENSURE_SMALL( loaded->scale.z - def.scale.z, FLT_EPSILON );
	ENSURE_SMALL( loaded->minHeight - def.globalMinimumHeight, FLT_EPSILON );
	ENSURE_SMALL( loaded->maxHeight - def.globalMaximumHeight, FLT_EPSILON );

	int cellCount = ( def.countX - 1 ) * ( def.countZ - 1 );
	for ( int i = 0; i < cellCount; ++i )
	{
		ENSURE( loaded->materialIndices[i] == materials[i] );
	}

	// Recovered heights round-trip within the quantization tolerance.
	float quantum = ( def.globalMaximumHeight - def.globalMinimumHeight ) / (float)UINT16_MAX;
	for ( int i = 0; i < def.countX * def.countZ; ++i )
	{
		float recovered = loaded->minHeight + loaded->heightScale * loaded->compressedHeights[i];
		ENSURE_SMALL( recovered - heights[i], 2.0f * quantum );
	}

	b3DestroyHeightField( loaded );
	return 0;
}

int HeightFieldTest( void )
{
	RUN_SUBTEST( HeightFieldCreate );
	RUN_SUBTEST( HeightFieldTriangleIndex );
	RUN_SUBTEST( HeightFieldWinding );
	RUN_SUBTEST( RayCastFlatField );
	RUN_SUBTEST( OverlapAtSurface );
	RUN_SUBTEST( FileRoundtrip );

	return 0;
}
