#pragma once

#include "LiteMath/LiteMath.h"

#include <ostream>

struct Colour
{
    uint8_t Blue;
    uint8_t Green;
    uint8_t Red;
    uint8_t Alpha;
};
static_assert (sizeof(Colour) == sizeof(uint32_t));


using Picture = std::vector<Colour>;

struct Ray
{
    LiteMath::float3 ray_pos;
    LiteMath::float3 ray_dir;
};

std::ostream& operator<<(std::ostream& stream, LiteMath::float3 vector);