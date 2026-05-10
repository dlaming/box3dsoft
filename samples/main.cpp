// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

// CRT's memory leak detection
#if defined( _WIN64 )
#ifndef _CRTDBG_MAP_ALLOC
#define _CRTDBG_MAP_ALLOC
#endif
#include <crtdbg.h>
#endif

// clang-format off
#include "glad/glad.h"
#include "GLFW/glfw3.h"
// clang-format on

#include "font.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include "renderer.h"
#include "sample.h"
#include "shader.h"

#include <stdio.h>

#ifdef TRACY_ENABLE
#include <client/TracyProfiler.hpp>
#endif

// imgui notes
// high-dpi support: https://github.com/ocornut/imgui/issues/5081
// linear blending:
// http://arkanis.de/weblog/2023-08-14-simple-good-quality-subpixel-text-rendering-in-opengl-with-stb-truetype-and-dual-source-blending

inline bool IsPowerOfTwo( int32_t x )
{
	return ( x != 0 ) && ( ( x & ( x - 1 ) ) == 0 );
}

void* AllocFcn( int32_t size, int32_t alignment )
{
	// Allocation must be a multiple of alignment or risk a seg fault
	// https://en.cppreference.com/w/c/memory/aligned_alloc
	assert( IsPowerOfTwo( alignment ) );
	size_t sizeAligned = ( ( size - 1 ) | ( alignment - 1 ) ) + 1;
	assert( ( sizeAligned & ( alignment - 1 ) ) == 0 );

#if defined( _MSC_VER )
	void* ptr = _aligned_malloc( sizeAligned, alignment );
#else
	void* ptr = aligned_alloc( alignment, sizeAligned );
#endif
	assert( ptr != nullptr );
	return ptr;
}

void FreeFcn( void* mem )
{
#if defined( _MSC_VER )
	_aligned_free( mem );
#else
	free( mem );
#endif
}

static int AssertHandler( const char* cond, const char* file, int line )
{
	printf( "Box3D Assert: %s, %s, line %d\n", cond, file, line );
	return 1;
}

#define WINDOW_TITLE "Box3D v0.1.0"

static int s_windowWidth = 1920;
static int s_windowHeight = 1080;
static int s_bufferWidth = s_windowWidth;
static int s_bufferHeight = s_windowHeight;

Font* s_font = nullptr;
SampleManager s_manager;

static void WindowSizeCallback( GLFWwindow* window, int width, int height )
{
	s_windowWidth = width;
	s_windowHeight = height;
	glfwGetFramebufferSize( window, &s_bufferWidth, &s_bufferHeight );
	s_manager.Resize( s_windowWidth, s_windowHeight, s_bufferWidth, s_bufferHeight );
}

static void ErrorCallback( int error, const char* description )
{
	// Report GLFW error
	printf( "GLFW Error %d: %s", error, description );
}

static void KeyCallback( GLFWwindow* window, int key, int scancode, int action, int mods )
{
	ImGui_ImplGlfw_KeyCallback( window, key, scancode, action, mods );
	if ( ImGui::GetIO().WantCaptureKeyboard )
	{
		return;
	}

	s_manager.Keyboard( key, action, mods );
}

void CharCallback( GLFWwindow* window, unsigned int character )
{
	ImGui_ImplGlfw_CharCallback( window, character );
}

static void MouseButtonCallback( GLFWwindow* window, int button, int action, int modifiers )
{
	ImGui_ImplGlfw_MouseButtonCallback( window, button, action, modifiers );
	if ( ImGui::GetIO().WantCaptureMouse )
	{
		return;
	}

	double xd, yd;
	glfwGetCursorPos( window, &xd, &yd );

	b3Vec2 ps = { float( xd ), float( yd ) };

	// Use the mouse to move things around.
	if ( button == GLFW_MOUSE_BUTTON_1 )
	{
		if ( action == GLFW_PRESS )
		{
			s_manager.m_sample->MouseDown( ps, button, modifiers );
		}

		if ( action == GLFW_RELEASE )
		{
			s_manager.m_sample->MouseUp( ps, button );
		}
	}
}

static void MouseMotionCallback( GLFWwindow* window, double x, double y )
{
	b3Vec2 ps = { float( x ), float( y ) };

	ImGui_ImplGlfw_CursorPosCallback( window, ps.x, ps.y );

	s_manager.m_sample->MouseMove( ps );
}

static void ScrollCallback( GLFWwindow* window, double deltaX, double deltaY )
{
	ImGui_ImplGlfw_ScrollCallback( window, deltaX, deltaY );
}

static void Startup( GLFWwindow* window )
{
	// Initialize font
	s_font = Font::Create( &s_manager.m_context.camera, "data/fonts/Roboto-Medium.ttf", 18.0f );

	if ( s_font == nullptr )
	{
		exit( EXIT_FAILURE );
	}

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();

	// Setup Platform/Renderer bindings
	ImGui_ImplGlfw_InitForOpenGL( window, false );
	ImGui_ImplOpenGL3_Init();

	ImGuiIO& io = ImGui::GetIO();
	io.Fonts->AddFontFromFileTTF( "data/fonts/Roboto-Medium.ttf", 16.0f );

	// Initialize test manager
	s_manager.Startup( window, s_windowWidth, s_windowHeight, s_bufferWidth, s_bufferHeight );
}

static void Render( GLFWwindow* window, float elapsedTime )
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();

	ImGui::NewFrame();

	glViewport( 0, 0, s_bufferWidth, s_bufferHeight );

	glClearColor( 0.3f, 0.3f, 0.3f, 1.0f );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

	s_manager.Render( window, elapsedTime );

	int y = s_windowHeight - 10;

	int triangleIndex = s_manager.m_sample->m_triangleIndex;
	uint64_t materialId = s_manager.m_sample->m_userMaterialId;

	const Camera* camera = &s_manager.m_context.camera;
	static char buffer[128];
	snprintf( buffer, 128, "%.1f ms - step %d - camera (%g, %g, %g, %g) - tri %d - mat %d", 1000.0f * elapsedTime, s_manager.m_sample->m_stepCount,
			  camera->m_yaw, camera->m_pitch, camera->m_radius, camera->m_speed, triangleIndex, (int)materialId );
	DrawText( 10, y, b3_colorLightGray, buffer );

	if ( s_font != nullptr )
	{
		s_font->Flush();
	}

	//ImGui::ShowDemoWindow();
	//ImPlot::ShowDemoWindow();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

	// Platform functions may have changed the current OpenGL context
	glfwMakeContextCurrent( window );
	glfwSwapBuffers( window );
}

static void Shutdown( GLFWwindow* window )
{
	// Shutdown test manager
	s_manager.Shutdown( window );

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();

	delete s_font;
	s_font = nullptr;
}

int main()
{
#if defined( _WIN64 )
	// Enable run-time memory check for debug builds
	// How to break at the leaking allocation, in the watch window enter this variable
	// and set it to the allocation number in {}. Do this at the first line in main.
	// https://learn.microsoft.com/en-us/troubleshoot/developer/visualstudio/cpp/libraries/use-rtbreakalloc-debug-memory-allocation
	//_CrtSetBreakAlloc(1392);
	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDOUT );
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif

#ifdef TRACY_ENABLE
	tracy::StartupProfiler();
#endif

	b3SetAllocator( AllocFcn, FreeFcn );
	b3SetAssertFcn( AssertHandler );

	// Initialize GLFW
	glfwSetErrorCallback( ErrorCallback );
	if ( !glfwInit() )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		return -1;
	}

	// Create OpenGL window
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 4 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 1 );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

	// MSAA
	glfwWindowHint( GLFW_SAMPLES, 4 );

	GLFWwindow* window = glfwCreateWindow( s_windowWidth, s_windowHeight, WINDOW_TITLE, nullptr, nullptr );
	if ( window == nullptr )
	{
		glfwTerminate();
		return -1;
	}

	glfwGetWindowSize( window, &s_windowWidth, &s_windowHeight );
	glfwGetFramebufferSize( window, &s_bufferWidth, &s_bufferHeight );

	// Set window callbacks
	glfwSetKeyCallback( window, KeyCallback );
	glfwSetCharCallback( window, CharCallback );
	glfwSetCursorPosCallback( window, MouseMotionCallback );
	glfwSetMouseButtonCallback( window, MouseButtonCallback );
	glfwSetScrollCallback( window, ScrollCallback );
	glfwSetWindowSizeCallback( window, WindowSizeCallback );

	// Make OpenGL context current
	glfwMakeContextCurrent( window );

	// This enables v-sync
	// glfwSwapInterval( 1 );

	// Setup GLAD bindings
	if ( !gladLoadGL() )
	{
		fprintf( stderr, "Failed to initialize glad\n" );
		glfwTerminate();
		return -1;
	}

	{
		const char* glVersionString = (const char*)glGetString( GL_VERSION );
		const char* glslVersionString = (const char*)glGetString( GL_SHADING_LANGUAGE_VERSION );
		printf( "OpenGL %s, GLSL %s\n", glVersionString, glslVersionString );
	}

	Startup( window );

	glEnable( GL_DEPTH_TEST );

	float frameTime = 0.0f;
	while ( !glfwWindowShouldClose( window ) )
	{
		glfwPollEvents();

		if (s_manager.m_context.minimized)
		{
			continue;
		}

		double time1 = glfwGetTime();

		s_manager.Step();
		Render( window, frameTime );

		// Limit frame rate to 60Hz
		double time2 = glfwGetTime();
		double targetTime = time1 + 1.0 / 60.0;
		while ( time2 < targetTime )
		{
			b3Yield();
			time2 = glfwGetTime();
		}

		frameTime = float( time2 - time1 );
	}

	Shutdown( window );

	glfwDestroyWindow( window );
	glfwTerminate();

#ifdef TRACY_ENABLE
	tracy::ShutdownProfiler();
#endif

#if defined( _WIN64 )
	_CrtDumpMemoryLeaks();
#endif

	return 0;
}
