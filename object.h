#pragma once

#include "common.h"

#include "LiteMath/LiteMath.h"

#include <variant>
#include <memory>

struct Hit 
{
    LiteMath::float3 hit_position;
    float t;
    LiteMath::float3 normal_dir;
    LiteMath::float3 albedo;
    Colour colour;
    bool is_reflective;
};

struct Miss 
{

};

using IntersectionResult = std::variant<Hit, Miss>;


struct IObject 
{
    virtual ~IObject() = default;
    virtual IntersectionResult intersect(Ray ray, bool debug) = 0;
};

std::unique_ptr<IObject> create_object_plane(float a, float b, float c, float d, Colour colour={255,0,0,0}, bool reflective=false);
std::unique_ptr<IObject> create_object_mesh(const char* file_name, Colour colour={0,255,0,0}, bool reflective=false);
std::unique_ptr<IObject> create_object_triangle(
    LiteMath::float3 A_pos,
    LiteMath::float3 B_pos,
    LiteMath::float3 C_pos,
    LiteMath::float3 A_norm_dir,
    LiteMath::float3 B_norm_dir,
    LiteMath::float3 C_norm_dir);