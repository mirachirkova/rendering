#pragma once

#include "common.h"
#include "object.h"

#include "LiteMath/LiteMath.h"

#include <memory>

struct IScene 
{
    virtual ~IScene() = default;
    virtual Colour trace_ray(Ray ray, bool shadows, bool debug) = 0;
};

std::unique_ptr<IScene> create_scene(std::vector<std::unique_ptr<IObject>> objects, 
    LiteMath::float3 light_position= {1.0f, 1.0f, -1.0f});