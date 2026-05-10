// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "arena_allocator.h"

#include "core.h"

#include <stdbool.h>
#include <stddef.h>

b3Stack b3CreateStack( int capacity )
{
	B3_ASSERT( capacity >= 0 );
	b3Stack stack = { 0 };
	stack.capacity = capacity;
	stack.memory = (char*)b3Alloc( capacity );
	return stack;
}

void b3DestroyStack( b3Stack* stack )
{
	b3Free( stack->memory, stack->capacity );
}

void* b3StackAlloc( b3Stack* stack, int size, const char* name )
{
	if ( stack->entryCount == B3_MAX_STACK_ENTRIES )
	{
		B3_ASSERT( false );
		return NULL;
	}

	// ensure allocation is 32 byte aligned to support 256-bit SIMD
	int size32 = ( ( size - 1 ) | 0x1F ) + 1;

	b3StackEntry entry;
	entry.size = size32;
	entry.name = name;
	if ( stack->index + size32 > stack->capacity )
	{
		// fall back to the heap (undesirable)
		entry.data = (char*)b3Alloc( size32 );
		entry.usedMalloc = true;

		B3_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}
	else
	{
		entry.data = stack->memory + stack->index;
		entry.usedMalloc = false;
		stack->index += size32;

		B3_ASSERT( ( (uintptr_t)entry.data & 0x1F ) == 0 );
	}

	stack->allocation += size32;
	if ( stack->allocation > stack->maxAllocation )
	{
		stack->maxAllocation = stack->allocation;
	}

	stack->entries[stack->entryCount] = entry;
	stack->entryCount += 1;
	return entry.data;
}

void b3StackFree( b3Stack* stack, void* mem )
{
	int entryCount = stack->entryCount;
	B3_ASSERT( entryCount > 0 );
	b3StackEntry* entry = stack->entries + ( entryCount - 1 );
	B3_ASSERT( mem == entry->data );
	if ( entry->usedMalloc )
	{
		b3Free( mem, entry->size );
	}
	else
	{
		stack->index -= entry->size;
	}
	stack->allocation -= entry->size;
	stack->entryCount -= 1;
}

void b3GrowStack( b3Stack* stack )
{
	// Stack must not be in use
	B3_ASSERT( stack->allocation == 0 );

	if ( stack->maxAllocation > stack->capacity )
	{
		b3Free( stack->memory, stack->capacity );
		stack->capacity = stack->maxAllocation + stack->maxAllocation / 2;
		stack->memory = (char*)b3Alloc( stack->capacity );
	}
}

int b3GetStackCapacity( b3Stack* stack )
{
	return stack->capacity;
}

int b3GetStackAllocation( b3Stack* stack )
{
	return stack->allocation;
}

int b3GetMaxStackAllocation( b3Stack* stack )
{
	return stack->maxAllocation;
}

b3Arena b3CreateArena( int capacity )
{
	int c = capacity > 8 ? capacity : 8;
	return (b3Arena){
		.memory = b3Alloc( c ),
		.capacity = c,
		.index = 0,
	};
}

void b3DestroyArena(b3Arena* arena)
{
	b3Free( arena->memory, arena->capacity );
	*arena = (b3Arena){ 0 };
}
