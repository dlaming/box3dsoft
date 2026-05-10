// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "utility.h"

#include "box3d/types.h"

uint32_t CreateProgramFromFiles( const char* vertexPath, const char* fragmentPath );
uint32_t CreateProgramFromStrings( const char* vs, const char* fs );

void DestroyShader( uint32_t programId );

// todo these are low value wrappers
int GetUniformLocation( uint32_t programId, const char* name );

void SetUniformBool( int location, bool value );
void SetUniformInt( int location, int value );
void SetUniformFloat( int location, float value );
void SetUniformVec3( int location, b3Vec3 value );
void SetUniformVec3( int location, float x, float y, float z );
void SetUniformVec4( int location, Vector4 value );
void SetUniformVec4( int location, float x, float y, float z, float w );
void SetUniformColor( int location, RGBAf value );
void SetUniformMatrix4( int location, const Matrix4& mat );

void SetUniformBool( uint32_t programId, const char* name, bool value );
void SetUniformInt( uint32_t programId, const char* name, int value );
void SetUniformFloat( uint32_t programId, const char* name, float value );
void SetUniformVec3( uint32_t programId, const char* name, b3Vec3 value );
void SetUniformVec3( uint32_t programId, const char* name, float x, float y, float z );
void SetUniformVec4( uint32_t programId, const char* name, Vector4 value );
void SetUniformVec4( uint32_t programId, const char* name, float x, float y, float z, float w );
void SetUniformColor( uint32_t programId, const char* name, RGBAf value );
void SetUniformMatrix4( uint32_t programId, const char* name, const Matrix4& mat );

void CheckOpenGL();
void DumpGLInfo( bool dumpExtensions );
