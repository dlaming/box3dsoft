// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once
#include "box3d/base.h"

#include <stdbool.h>
#include <stddef.h>

#define B3_MAX_STACK_ENTRIES 32

typedef struct b3StackEntry
{
	char* data;
	const char* name;
	int size;
	bool usedMalloc;
} b3StackEntry;

// This is a stack-like arena allocator used for fast per step allocations.
// You must nest allocate/free pairs. The code will B3_ASSERT
// if you try to interleave multiple allocate/free pairs.
// This allocator uses the heap if space is insufficient.
// I could remove the need to free entries individually.
typedef struct b3Stack
{
	char* memory;
	int capacity;
	int index;

	int allocation;
	int maxAllocation;

	b3StackEntry entries[B3_MAX_STACK_ENTRIES];
	int entryCount;
} b3Stack;

typedef struct b3Arena
{
	char* memory;
	int capacity;
	int index;
} b3Arena;

#ifdef __cplusplus
extern "C"
{
#endif

b3Stack b3CreateStack( int capacity );
void b3DestroyStack( b3Stack* stack );

void* b3StackAlloc( b3Stack* stack, int size, const char* name );
void b3StackFree( b3Stack* stack, void* mem );

// Grow the stack based on usage
void b3GrowStack( b3Stack* stack );

int b3GetStackCapacity( b3Stack* stack );
int b3GetStackAllocation( b3Stack* stack );
int b3GetMaxStackAllocation( b3Stack* stack );

b3Arena b3CreateArena( int capacity );
void b3DestroyArena( b3Arena* arena );

static inline void* b3Bump( b3Arena* arena, int size )
{
	if (size == 0)
	{
		return NULL;
	}

	if ( size + arena->index > arena->capacity )
	{
		B3_ASSERT( false );
		return NULL;
	}

	void* memory = arena->memory + arena->index;
	arena->index += size;
	return memory;
}

#ifdef __cplusplus
}
#endif
