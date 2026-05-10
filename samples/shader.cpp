// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER ) && !defined( _CRT_SECURE_NO_WARNINGS )
	#define _CRT_SECURE_NO_WARNINGS
#endif

#include "shader.h"

#include <assert.h>
#include <glad/glad.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <string>

void CheckOpenGL()
{
	GLenum errCode = glGetError();
	if ( errCode != GL_NO_ERROR )
	{
		fprintf( stderr, "OpenGL error = %d\n", errCode );
		assert( false );
	}
}

void DumpGLInfo( bool dumpExtensions )
{
	const GLubyte* renderer = glGetString( GL_RENDERER );
	const GLubyte* vendor = glGetString( GL_VENDOR );
	const GLubyte* version = glGetString( GL_VERSION );
	const GLubyte* glslVersion = glGetString( GL_SHADING_LANGUAGE_VERSION );

	int major, minor;
	glGetIntegerv( GL_MAJOR_VERSION, &major );
	glGetIntegerv( GL_MINOR_VERSION, &minor );

	printf( "-------------------------------------------------------------\n" );
	printf( "GL Vendor    : %s\n", vendor );
	printf( "GL Renderer  : %s\n", renderer );
	printf( "GL Version   : %s\n", version );
	printf( "GL Version   : %d.%d\n", major, minor );
	printf( "GLSL Version : %s\n", glslVersion );
	printf( "-------------------------------------------------------------\n" );

	if ( dumpExtensions )
	{
		int nExtensions;
		glGetIntegerv( GL_NUM_EXTENSIONS, &nExtensions );
		for ( int i = 0; i < nExtensions; i++ )
		{
			printf( "%s\n", glGetStringi( GL_EXTENSIONS, i ) );
		}
	}
}

static void PrintLog( uint32_t object )
{
	int length = 0;
	if ( glIsShader( object ) )
	{
		glGetShaderiv( object, GL_INFO_LOG_LENGTH, &length );
	}
	else if ( glIsProgram( object ) )
	{
		glGetProgramiv( object, GL_INFO_LOG_LENGTH, &length );
	}
	else
	{
		fprintf( stderr, "printlog: Not a shader or a program\n" );
		return;
	}

	char* log = static_cast<char*>( malloc( length * sizeof( char ) ) );

	if ( glIsShader( object ) )
	{
		glGetShaderInfoLog( object, length, nullptr, log );
	}
	else if ( glIsProgram( object ) )
	{
		glGetProgramInfoLog( object, length, nullptr, log );
	}

	fprintf( stderr, "%s", log );
	free( log );
}

static uint32_t CreateShaderFromString( const char* source, GLenum type )
{
	uint32_t shader = glCreateShader( type );
	const char* sources[] = { source };

	glShaderSource( shader, 1, sources, nullptr );
	glCompileShader( shader );

	int success = GL_FALSE;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &success );

	if ( success == GL_FALSE )
	{
		fprintf( stderr, "Error compiling shader of type %d!\n", type );
		PrintLog( shader );
	}

	return shader;
}

static bool ReadFile( const char* filename, std::string& out )
{
	FILE* file = fopen( filename, "rb" );
	if ( file == nullptr )
	{
		return false;
	}

	fseek( file, 0, SEEK_END );
	long size = ftell( file );
	fseek( file, 0, SEEK_SET );

	out.resize( (size_t)size );
	if ( size > 0 )
	{
		fread( &out[0], 1, (size_t)size, file );
	}
	fclose( file );
	return true;
}

static std::string DirectoryOf( const char* path )
{
	const char* lastSep = nullptr;
	for ( const char* p = path; *p; ++p )
	{
		if ( *p == '/' || *p == '\\' )
		{
			lastSep = p;
		}
	}
	return lastSep ? std::string( path, lastSep + 1 ) : std::string();
}

// Substitutes lines of the form `#include "name"` with the contents of that
// file (resolved relative to `baseDir`). Recursive but not cyclic-safe — keep
// the include graph a DAG. Emits `#line` directives so compile errors keep
// readable line numbers in both the parent and the included file.
static bool ResolveIncludes( const std::string& source, const std::string& baseDir, std::string& out )
{
	out.reserve( out.size() + source.size() * 2 );

	int lineNo = 1;
	size_t i = 0;
	while ( i < source.size() )
	{
		size_t lineStart = i;
		size_t lineEnd = source.find( '\n', i );
		if ( lineEnd == std::string::npos )
		{
			lineEnd = source.size();
		}

		size_t t = lineStart;
		while ( t < lineEnd && ( source[t] == ' ' || source[t] == '\t' ) )
		{
			t++;
		}

		bool isInclude = false;
		std::string includeName;
		if ( lineEnd >= t + 9 && source.compare( t, 8, "#include" ) == 0 &&
			 ( source[t + 8] == ' ' || source[t + 8] == '\t' ) )
		{
			size_t q1 = source.find( '"', t + 8 );
			size_t q2 = ( q1 != std::string::npos && q1 < lineEnd ) ? source.find( '"', q1 + 1 ) : std::string::npos;
			if ( q1 != std::string::npos && q2 != std::string::npos && q2 < lineEnd )
			{
				isInclude = true;
				includeName = source.substr( q1 + 1, q2 - q1 - 1 );
			}
		}

		if ( isInclude )
		{
			std::string includePath = baseDir + includeName;
			std::string includedRaw;
			if ( !ReadFile( includePath.c_str(), includedRaw ) )
			{
				fprintf( stderr, "Cannot open include '%s'\n", includePath.c_str() );
				return false;
			}

			char hdr[64];
			snprintf( hdr, sizeof( hdr ), "#line 1\n" );
			out += hdr;

			if ( !ResolveIncludes( includedRaw, DirectoryOf( includePath.c_str() ), out ) )
			{
				return false;
			}

			if ( !out.empty() && out.back() != '\n' )
			{
				out += '\n';
			}

			snprintf( hdr, sizeof( hdr ), "#line %d\n", lineNo + 1 );
			out += hdr;
		}
		else
		{
			out.append( source, lineStart, lineEnd - lineStart );
			out += '\n';
		}

		i = ( lineEnd < source.size() ) ? lineEnd + 1 : lineEnd;
		lineNo++;
	}
	return true;
}

static uint32_t CreateShaderFromFile( const char* filename, GLenum type )
{
	std::string raw;
	if ( !ReadFile( filename, raw ) )
	{
		fprintf( stderr, "Error opening %s\n", filename );
		return 0;
	}

	std::string source;
	if ( !ResolveIncludes( raw, DirectoryOf( filename ), source ) )
	{
		fprintf( stderr, "Error resolving includes in %s\n", filename );
		return 0;
	}

	uint32_t shader = glCreateShader( type );
	const char* sources[] = { source.c_str() };

	glShaderSource( shader, 1, sources, nullptr );
	glCompileShader( shader );

	int success = GL_FALSE;
	glGetShaderiv( shader, GL_COMPILE_STATUS, &success );

	if ( success == GL_FALSE )
	{
		fprintf( stderr, "Error compiling shader of type %d in %s!\n", type, filename );
		PrintLog( shader );
	}

	return shader;
}

uint32_t CreateProgramFromFiles( const char* vertexPath, const char* fragmentPath )
{
	uint32_t vertex = CreateShaderFromFile( vertexPath, GL_VERTEX_SHADER );
	if ( vertex == 0 )
	{
		return 0;
	}

	uint32_t fragment = CreateShaderFromFile( fragmentPath, GL_FRAGMENT_SHADER );
	if ( fragment == 0 )
	{
		return 0;
	}

	uint32_t program = glCreateProgram();
	glAttachShader( program, vertex );
	glAttachShader( program, fragment );

	glLinkProgram( program );

	int success = GL_FALSE;
	glGetProgramiv( program, GL_LINK_STATUS, &success );
	if ( success == GL_FALSE )
	{
		fprintf( stderr, "glLinkProgram:" );
		PrintLog( program );
		return 0;
	}

	glDeleteShader( vertex );
	glDeleteShader( fragment );

	return program;
}

uint32_t CreateProgramFromStrings( const char* vs, const char* fs )
{
	uint32_t vertex = CreateShaderFromString( vs, GL_VERTEX_SHADER );
	if ( vertex == 0 )
	{
		return 0;
	}

	uint32_t fragment = CreateShaderFromString( fs, GL_FRAGMENT_SHADER );
	if ( fragment == 0 )
	{
		return 0;
	}

	uint32_t programId = glCreateProgram();
	glAttachShader( programId, vertex );
	glAttachShader( programId, fragment );

	glLinkProgram( programId );

	int success = GL_FALSE;
	glGetProgramiv( programId, GL_LINK_STATUS, &success );
	if ( success == GL_FALSE )
	{
		fprintf( stderr, "glLinkProgram:" );
		PrintLog( programId );
		return 0;
	}

	glDeleteShader( vertex );
	glDeleteShader( fragment );

	return programId;
}

void DestroyShader( uint32_t programId )
{
	glDeleteProgram( programId );
}

int GetUniformLocation( uint32_t programId, const char* name )
{
	return glGetUniformLocation( programId, name );
}

void SetUniformBool( int location, bool value )
{
	glUniform1i( location, (int)value );
}

void SetUniformInt( int location, int value )
{
	glUniform1i( location, value );
}

void SetUniformFloat( int location, float value )
{
	glUniform1f( location, value );
}

void SetUniformVec3( int location, b3Vec3 value )
{
	glUniform3fv( location, 1, &value.x );
}

void SetUniformVec3( int location, float x, float y, float z )
{
	glUniform3f( location, x, y, z );
}

void SetUniformVec4( int location, Vector4 value )
{
	glUniform4fv( location, 1, &value.x );
}

void SetUniformColor( int location, RGBAf value )
{
	glUniform4fv( location, 1, &value.r );
}

void SetUniformVec4( int location, float x, float y, float z, float w )
{
	glUniform4f( location, x, y, z, w );
}

void SetUniformMatrix4( int location, const Matrix4& mat )
{
	glUniformMatrix4fv( location, 1, GL_FALSE, &mat.cx.x );
}

void SetUniformBool( uint32_t programId, const char* name, bool value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform1i( location, (int)value );
	}
	else
	{
		location += 0;
	}
}

void SetUniformInt( uint32_t programId, const char* name, int value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform1i( location, value );
	}
}

void SetUniformFloat( uint32_t programId, const char* name, float value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform1f( location, value );
	}
}

void SetUniformVec3( uint32_t programId, const char* name, b3Vec3 value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform3fv( location, 1, &value.x );
	}
}

void SetUniformVec3( uint32_t programId, const char* name, float x, float y, float z )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform3f( location, x, y, z );
	}
}

void SetUniformVec4( uint32_t programId, const char* name, Vector4 value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform4fv( location, 1, &value.x );
	}
}

void SetUniformVec4( uint32_t programId, const char* name, float x, float y, float z, float w )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform4f( location, x, y, z, w );
	}
}

void SetUniformColor( uint32_t programId, const char* name, RGBAf value )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniform4fv( location, 1, &value.r );
	}
}

void SetUniformMatrix4( uint32_t programId, const char* name, const Matrix4& mat )
{
	int location = glGetUniformLocation( programId, name );
	if ( location != -1 )
	{
		glUniformMatrix4fv( location, 1, GL_FALSE, &mat.cx.x );
	}
}
