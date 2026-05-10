// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

struct SsaoBuffer;
struct GeoBuffer;

SsaoBuffer* CreateSsaoBuffer(int32_t width, int32_t height);

void BindSsaoBuffer(SsaoBuffer* buffer);
void SetSsaoSamples(SsaoBuffer* buffer, uint32_t ssaoShader);
void BindSsaoTextures(SsaoBuffer* buffer);

void BindSsaoBlurBuffer(SsaoBuffer* buffer);
void BindSsaoColorTexture(SsaoBuffer* buffer);
void BindSsaoBlurTexture(SsaoBuffer* buffer);

void DestroySsaoBuffer(SsaoBuffer* buffer);
