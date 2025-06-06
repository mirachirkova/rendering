#pragma once

#include "common.h"
#include "scene.h"

#include "LiteMath/LiteMath.h"

#include <cstdint>
#include <vector>
#include <memory>

struct ICamera
{
    virtual ~ICamera() = default;
    virtual void take_picture(Picture &pixels, bool shadows) = 0;
    virtual void move_forward() = 0;
    virtual void move_backward() = 0;
    virtual void move_left() = 0;
    virtual void move_right() = 0;
    virtual void move_up() = 0;
    virtual void move_down() = 0;
    virtual void zoom_in() = 0;
    virtual void zoom_out() = 0;
    virtual void update_zoom(float new_coef) = 0;
};

std::unique_ptr<ICamera> create_camera(
    std::unique_ptr<IScene> scene, 
    uint32_t picture_width, 
    uint32_t picture_height, 
    LiteMath::float3 camera_position = {0.0f,0.0f,0.0f});

/*const Colour p{};
static const auto a = *reinterpret_cast<const uint32_t*>(&p);*/