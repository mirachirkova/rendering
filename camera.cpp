#include "camera.h"

#include <iostream>

namespace {

class Camera
    : public ICamera
{
public:
    Camera(std::unique_ptr<IScene> scene, uint32_t picture_width, uint32_t picture_height, LiteMath::float3 camera_position)
        : m_scene(std::move(scene))
        , m_picture_width(picture_width)
        , m_picture_height(picture_height)
        , m_camera_position(camera_position)
    { }

    void take_picture(Picture &pixels, bool shadows) override
    {
        for (uint32_t y = 0; y < m_picture_height; ++y) {
            for (uint32_t x = 0; x < m_picture_width; ++x) {
                Ray ray;
                ray.ray_pos = m_camera_position;
                ray.ray_dir = LiteMath::normalize(LiteMath::float3((x - m_picture_width / 2.0) * m_zoom_coeff, (m_picture_height / 2.0 - y) * m_zoom_coeff, 100.0f * m_zoom_coeff) - m_camera_position);;
                bool debug = x == 300 && y == 200;
                pixels[y * m_picture_width + x] = m_scene->trace_ray(ray, shadows, debug);
            }
        }
    }

    void move_forward() override
    {
        m_camera_position.z += 1.0f;
        std::cerr << "moved forward: " << m_camera_position << std::endl;
    }

    void move_backward() override
    {
        m_camera_position.z -= 1.0f;
        std::cerr << "moved backward: " << m_camera_position << std::endl;
    }

    void move_left() override
    {
        m_camera_position.x -= 1.0f;
        std::cerr << "moved left: " << m_camera_position << std::endl;
    }

    void move_right() override
    {
        m_camera_position.x += 1.0f;
        std::cerr << "moved right: " << m_camera_position << std::endl;
    }

    void move_up() override
    {
        m_camera_position.y += 1.0f;
        std::cerr << "moved up: " << m_camera_position << std::endl;
    }

    void move_down() override
    {
        m_camera_position.y -= 1.0f;
        std::cerr << "moved down: " << m_camera_position << std::endl;
    }

    void zoom_in() override
    {
        m_zoom_coeff *= 0.5f;
    }

    void zoom_out() override
    {
        m_zoom_coeff /= 0.5f;
    }

    void update_zoom(float new_coef) override
    {
        m_zoom_coeff = new_coef;
    }
private:
    Colour get_pixel()
    {
        return {};
    }

private:
    std::unique_ptr<IScene> m_scene;
    const uint32_t m_picture_width;
    const uint32_t m_picture_height;
    float m_zoom_coeff = 1.00f;
    LiteMath::float3 m_camera_position;
};

} // namespace

std::unique_ptr<ICamera> create_camera(std::unique_ptr<IScene> scene, uint32_t picture_width, uint32_t picture_height, LiteMath::float3 camera_position)
{
    return std::make_unique<Camera>(std::move(scene), picture_width, picture_height, camera_position);
}