// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#include "ssao_buffer.h"

#include "box3d/math_functions.h"
#include "shader.h"

#include <glad/glad.h>
#include <stdlib.h>
#include <random>
#include <stdint.h>
#include <stdio.h>

#include "geo_buffer.h"

struct SsaoBuffer
{
	int32_t width, height;
	uint32_t frameBufferId;
	uint32_t blurFrameBufferId;
	uint32_t colorBufferId;
	uint32_t blurColorBufferId;

	b3Vec3 ssaoKernel[32];
	uint32_t noiseTexture;
};

SsaoBuffer* CreateSsaoBuffer(int32_t width, int32_t height)
{
	SsaoBuffer* buffer = (SsaoBuffer*)malloc(sizeof(SsaoBuffer));
	buffer->width = width;
	buffer->height = height;

	// frame buffer
	glGenFramebuffers(1, &buffer->frameBufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->frameBufferId);

	// color buffer
	glGenTextures(1, &buffer->colorBufferId);
	glBindTexture(GL_TEXTURE_2D, buffer->colorBufferId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, buffer->colorBufferId, 0);

	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		fprintf(stderr, "SSAO Framebuffer not complete!\n");
		free(buffer);
		return nullptr;
	}

	// blur frame buffer
	glGenFramebuffers(1, &buffer->blurFrameBufferId);
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->blurFrameBufferId);

	// blur color buffer
	glGenTextures(1, &buffer->blurColorBufferId);
	glBindTexture(GL_TEXTURE_2D, buffer->blurColorBufferId);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, width, height, 0, GL_RED, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, buffer->blurColorBufferId, 0);
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
	{
		fprintf(stderr, "SSAO Blur Framebuffer not complete!\n");
		free(buffer);
		return nullptr;
	}

	// unbind frame buffer
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// generate sample kernel
	std::uniform_real_distribution<float> randomFloats(0.0f, 1.0f); // generates random floats between 0.0 and 1.0
	std::default_random_engine generator;

	for (uint32_t i = 0; i < 32; ++i)
	{
		b3Vec3 sample{randomFloats(generator) * 2.0f - 1.0f, randomFloats(generator) * 2.0f - 1.0f, randomFloats(generator)};
		sample = b3Normalize(sample);
		sample = randomFloats(generator) * sample;
		float scale = float(i) / 32.0f;

		// scale samples s.t. they're more aligned to center of kernel
		scale = b3LerpFloat(0.1f, 1.0f, scale * scale);
		sample = scale * sample;
		buffer->ssaoKernel[i] = sample;
	}

	// random sample kernel tangent vectors stuffed into a texture
	b3Vec3 ssaoNoise[16];
	for (b3Vec3& noise : ssaoNoise)
	{
		// rotate around z-axis (in tangent space)
		noise = {randomFloats(generator) * 2.0f - 1.0f, randomFloats(generator) * 2.0f - 1.0f, 0.0f};
	}

	glGenTextures(1, &buffer->noiseTexture);
	glBindTexture(GL_TEXTURE_2D, buffer->noiseTexture);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 4, 4, 0, GL_RGB, GL_FLOAT, &ssaoNoise[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	return buffer;
}

void BindSsaoBuffer(SsaoBuffer* buffer)
{
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->frameBufferId);
	glClear(GL_COLOR_BUFFER_BIT);
}

void SetSsaoSamples(SsaoBuffer* buffer, uint32_t ssaoShader)
{
	// Send kernel + rotation
	for (uint32_t i = 0; i < 32; ++i)
	{
		char sampleName[32];
		snprintf(sampleName, 32,  "samples[%d]", i);
		SetUniformVec3(ssaoShader, sampleName, buffer->ssaoKernel[i]);
	}
}

void BindSsaoTextures(SsaoBuffer* buffer)
{
	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, buffer->noiseTexture);
}

void BindSsaoBlurBuffer(SsaoBuffer* buffer)
{
	glBindFramebuffer(GL_FRAMEBUFFER, buffer->blurFrameBufferId);
	glClear(GL_COLOR_BUFFER_BIT);
}

void BindSsaoColorTexture(SsaoBuffer* buffer)
{
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, buffer->colorBufferId);
}

void BindSsaoBlurTexture(SsaoBuffer* buffer)
{
	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, buffer->blurColorBufferId);
}

void DestroySsaoBuffer(SsaoBuffer* buffer)
{
	glDeleteFramebuffers(1, &buffer->frameBufferId);
	glDeleteFramebuffers(1, &buffer->blurFrameBufferId);

	glDeleteTextures(1, &buffer->colorBufferId);
	glDeleteTextures(1, &buffer->blurColorBufferId);

	glDeleteTextures(1, &buffer->noiseTexture);

	free(buffer);
}
