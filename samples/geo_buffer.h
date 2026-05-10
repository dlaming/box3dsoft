// SPDX-FileCopyrightText: 2025 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

struct GeoBuffer;

GeoBuffer* CreateGeoBuffer(int32_t width, int32_t height);
void DestroyGeoBuffer(GeoBuffer* buffer);

void BindGeoBuffer(GeoBuffer* buffer);
//void tbBindGeoPositionNormal(GeoBuffer* buffer);
void BindGeoTextures(GeoBuffer* buffer);
void BindGeoPosition(GeoBuffer* buffer, int textureUnit);
void BindGeoDepthTexture(GeoBuffer* buffer, int textureUnit);
