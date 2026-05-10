// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "utility.h"

#include "box3d/types.h"

struct GLFWwindow;

struct PickRay
{
	b3Vec3 origin;
	b3Vec3 translation;
};

struct Camera
{
	Camera();

	// Window
	void Resize( int width, int height );

	// View
	b3Vec3 GetRight() const;
	b3Vec3 GetUp() const;
	b3Vec3 GetForward() const;
	b3Vec3 GetPosition() const;

	void Frame( b3AABB bounds);
	void SetView( float yaw, float pitch, float radius, b3Vec3 pivot = b3Vec3_zero );

	Matrix4 GetWorldMatrix() const;
	Matrix4 GetViewMatrix() const;

	// Projection
	Matrix4 GetProjectionMatrix() const;
	PickRay BuildPickRay( float x, float y) const;
	bool WorldToScreen( float& xScreen, float& yScreen, b3Vec3 worldPoint) const;

	// Control
	void Update( GLFWwindow* window, float elapsedTime);

	void UpdateTransform();

	// Window
	int m_width;
	int m_height;

	// Frame buffer
	int m_bufferWidth;
	int m_bufferHeight;

	// View yaw in degrees
	float m_yaw;

	// View pitch in degrees
	float m_pitch;

	float m_radius;
	b3Vec3 m_pivot;
	float m_speed;

	// Projection
	float m_fovY;
	float m_tanY;
	float m_ratio;
	float m_nearPlane;
	float m_farPlane;

	float m_nearWidth;
	float m_nearHeight;

	float m_farWidth;
	float m_farHeight;

	// World coordinates
	b3Vec3 m_right;
	b3Vec3 m_up;
	b3Vec3 m_forward;
	b3Vec3 m_position;

	bool m_thirdPerson;
};
