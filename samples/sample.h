// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "camera.h"

class Font;
struct MouseEvent;
struct MouseMoveEvent;
struct MouseWheelEvent;
struct Scene;
struct GLFWwindow;

struct SampleContext
{
	void Save();
	void Load();

	GLFWwindow* window = nullptr;
	Camera camera;
	Arena arena;
	Scene* scene = nullptr;
	bool minimized = false;
	class Sample* sample = nullptr;
	b3Capacity capacity;
	b3DebugDraw debugDraw;

	int windowWidth = 1920;
	int windowHeight = 1080;

	// Used to shift the origin and test round-off problems
	b3Vec3 origin = b3Vec3_zero;

	float hertz = 60.0f;
	float recycleDistance = 0.05f;
	int subStepCount = 4;
	int workerCount = 1;
	bool transparentDynamic = false;
	bool drawCounters = false;
	bool drawProfile = false;
	bool frameTime = false;
	bool transparent = false;
	bool enableWarmStarting = true;
	bool enableContinuous = true;
	bool enableSleep = true;
	bool pause = false;
	int singleStep = 0;
	bool restart = false;

	int sampleIndex = 0;
};

class Sample
{
public:
	explicit Sample( SampleContext* context );
	virtual ~Sample();

	void CreateWorld( b3Capacity* capacity );

	// Update and render are split to support pausing the simulation
	virtual void Step();

	virtual void Render();

	virtual void UpdateUI();

	virtual void Keyboard( int key, int action, int modifiers )
	{
	}

	virtual void MouseDown( b3Vec2 p, int button, int modifiers );
	virtual void MouseUp( b3Vec2 p, int button );
	virtual void MouseMove( b3Vec2 p );

	void ToggleThirdPerson();
	void DrawTextLine( const char* text, ... );
	void ResetProfile();

#if defined( NDEBUG )
	static constexpr bool m_isDebug = false;
#else
	static constexpr bool m_isDebug = true;
#endif

	static constexpr int m_maxTasks = 64;
	static constexpr int m_maxThreads = 64;
	static constexpr int m_profileCapacity = 512;

	SampleContext* m_context;
	GLFWwindow* m_window;
	Scene* m_scene;
	Camera* m_camera;

	b3WorldId m_worldId;
	b3Vec3 m_mousePoint;
	b3BodyId m_mouseBodyId;
	b3JointId m_mouseJointId;
	float m_mouseFraction;
	float m_mouseForceScale;
	float m_launchSpeedScale;
	int m_stepCount;
	int m_textLine;
	int m_textIncrement;
	int m_triangleIndex;
	uint64_t m_userMaterialId;
	
	b3Profile m_profiles[m_profileCapacity];
	int m_currentProfileIndex;
	int m_profileReadIndex;
	int m_profileWriteIndex;
	
	b3Vec2 m_mouseLast;
	b3Vec2 m_mouseDelta;
	bool m_didStep;
	bool m_stepWhilePaused;
	bool m_haveMouseLast;
};

#define MAX_SAMPLES 256

typedef Sample* SampleCreateFcn( SampleContext* context );

struct SampleEntry
{
	const char* Category;
	const char* Name;
	SampleCreateFcn* CreateFcn;
};

class SampleManager
{
public:
	static int Register( const char* category, const char* name, SampleCreateFcn* fcn );

	void Startup( GLFWwindow* window, int width, int height, int bufferWidth, int bufferHeight );
	void Step();
	void Resize( int width, int height, int bufferWidth, int bufferHeight );
	void Render( GLFWwindow* window, float elapsedTime );
	void Shutdown( GLFWwindow* window );

	void Keyboard( int key, int action, int modifiers );

	void CreateSample();

	void RenderScene( GLFWwindow* window, float elapsedTime );
	void UpdateUI( GLFWwindow* window );

	static SampleEntry sEntries[MAX_SAMPLES];
	static int sEntryCount;

	SampleContext m_context;
	Sample* m_sample = nullptr;
	bool m_showMenu = true;
};

struct CastClosestContext
{
	b3ShapeId shapeId;
	b3Vec3 point;
	b3Vec3 normal;
	float fraction;
	uint64_t materialId;
	int childIndex;
	int triangleIndex;
	bool hit;
};

float CastClosestCallback( b3ShapeId shapeId, b3Vec3 point, b3Vec3 normal, float fraction, uint64_t materialId,
							  int triangleIndex, int childIndex, void* context );

struct MoverShapeUserData
{
	float maxPush;
	bool clipVelocity;
};

struct PlaneExtra
{
	b3Vec3 point;
	b3ShapeId shapeId;
};

struct CharacterMover
{
	static constexpr int m_planeCapacity = 8;
	static constexpr float m_jumpSpeed = 5.0f;
	static constexpr float m_maxSpeed = 6.0f;
	static constexpr float m_minSpeed = 0.01f;
	static constexpr float m_stopSpeed = 1.0f;
	static constexpr float m_accelerate = 30.0f;
	static constexpr float m_friction = 4.0f;
	static constexpr float m_gravity = 15.0f;

	void Initialize( Sample* sample, b3Vec3 position );
	void SolveMove( float timeStep, b3Vec3 forward, b3Vec3 right, b3Vec2 throttle, bool clipVelocity );
	void Step( b3ShapeId* ignoreShapes, int ignoreCount, bool clipVelocity );

	Sample* m_sample;
	b3Transform m_transform;
	b3Vec3 m_velocity;
	b3Capsule m_capsule;
	b3CollisionPlane m_planes[m_planeCapacity] = {};
	PlaneExtra m_planeExtras[m_planeCapacity] = {};
	int m_planeCount;
	int m_totalIterations;
	float m_pogoVelocity;
	bool m_onGround;
	bool m_sprint;

	// Transient
	b3ShapeId* m_ignoreShapeIds;
	int m_ignoreCount;
};
