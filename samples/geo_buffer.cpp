// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "geo_buffer.h"

#include <glad/glad.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

struct GeoBuffer
{
	int32_t width, height;
	uint32_t frameBufferId;
	uint32_t positionId;
	uint32_t normalId;
	uint32_t albedoId;
	uint32_t depthTextureId;
};

GeoBuffer* CreateGeoBuffer(int32_t width, int32_t height)
{
	GeoBuffer* buffer = new GeoBuffer;

	buffer->width = width;
	buffer->height = height;

	glGenFramebuffers(1, &buffer->frameBufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->frameBufferId);

	// position color buffer
	glGenTextures(1, &buffer->positionId);
	glBindTexture(GL_TEXTURE_2D, buffer->positionId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, buffer->positionId, 0);

	// normal color buffer
	glGenTextures(1, &buffer->normalId);
	glBindTexture(GL_TEXTURE_2D, buffer->normalId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, buffer->normalId, 0);

	// color + specular color buffer
	glGenTextures(1, &buffer->albedoId);
	glBindTexture(GL_TEXTURE_2D, buffer->albedoId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, buffer->albedoId, 0);

	// tell OpenGL which color attachments we'll use (of this framebuffer) for rendering
	const unsigned int attachments[3] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2};
	glDrawBuffers(3, attachments);

	// create and attach depth buffer as a texture
	// Using a texture instead of a renderbuffer avoids a slow blit path on Apple Silicon
	glGenTextures(1, &buffer->depthTextureId);
	glBindTexture(GL_TEXTURE_2D, buffer->depthTextureId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, buffer->depthTextureId, 0);

	// finally check if framebuffer is complete
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		fprintf(stderr, "Failed to initialize glad\n");
		free(buffer);
		return nullptr;
	}

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	return buffer;
}

void BindGeoBuffer(GeoBuffer* buffer)
{
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->frameBufferId);

	// Clear each attachment to zero independently of the global glClearColor, so the
	// deferred lighting pass can identify sky pixels by gPosition.xyz == 0.
	static const GLfloat zero[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	glClearBufferfv(GL_COLOR, 0, zero);
	glClearBufferfv(GL_COLOR, 1, zero);
	glClearBufferfv(GL_COLOR, 2, zero);
	glClear(GL_DEPTH_BUFFER_BIT);
}

#if 0
void tbBindGeoPositionNormal(GeoBuffer* buffer)
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, buffer->positionId);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, buffer->normalId);
}
#endif

void BindGeoTextures(GeoBuffer* buffer)
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, buffer->positionId);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, buffer->normalId);
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, buffer->albedoId);
}

void BindGeoPosition(GeoBuffer* buffer, int textureUnit)
{
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, buffer->positionId);
}

void BindGeoDepthTexture(GeoBuffer* buffer, int textureUnit)
{
	glActiveTexture(GL_TEXTURE0 + textureUnit);
	glBindTexture(GL_TEXTURE_2D, buffer->depthTextureId);
}

#if 0
void tbResize(GeoBuffer* buffer, int32_t width, int32_t height)
{
	glBindTexture(GL_TEXTURE_2D, buffer->positionId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);

	glBindTexture(GL_TEXTURE_2D, buffer->normalId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, width, height, 0, GL_RGBA, GL_FLOAT, nullptr);

	glBindTexture(GL_TEXTURE_2D, buffer->albedoId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
}
#endif

void DestroyGeoBuffer(GeoBuffer* buffer)
{
	glDeleteFramebuffers(1, &buffer->frameBufferId);
	glDeleteTextures(1, &buffer->positionId);
	glDeleteTextures(1, &buffer->normalId);
	glDeleteTextures(1, &buffer->albedoId);
	glDeleteTextures(1, &buffer->depthTextureId);

	delete buffer;
}
