#include "scene.h"


namespace {

class Scene 
    : public IScene 
{
public:
    Scene(std::vector<std::unique_ptr<IObject>> objects, LiteMath::float3 light_position)
        : m_objects (std::move(objects))
        , m_light_position(light_position)
    { }

    Colour trace_ray(Ray ray, bool shadows,  bool debug) override
    {
        std::vector<IntersectionResult> intersection_objects (m_objects.size());
        for (int i = 0; i < m_objects.size(); ++i) {
            intersection_objects[i] = m_objects[i]->intersect(ray, debug);
        }

        Hit closest;
        closest.t = std::numeric_limits<float>::max();
        for (const auto& intersection_object : intersection_objects) {
            if (auto* hit = std::get_if<Hit>(&intersection_object); hit && hit->t < closest.t) {
                closest = *hit;
            }
        }

        if (closest.t != std::numeric_limits<float>::max()) {
            LiteMath::float3 to_light_dir = LiteMath::normalize(m_light_position - closest.hit_position);
            if (shadows && is_in_shadow({closest.hit_position, to_light_dir}, debug)) {
                return m_shadow_colour;
            }
            if (closest.is_reflective) {
                LiteMath::float3 reflect_ray_dir = LiteMath::normalize(ray.ray_dir - 2 * LiteMath::dot(ray.ray_dir, 
                    closest.normal_dir) * closest.normal_dir);
                return trace_ray({closest.hit_position, reflect_ray_dir}, shadows, debug);
            }
            return light_by_Lambert(to_light_dir, 
                closest.normal_dir, closest.colour);
        }
        //std::cout << "Miss!" << std::endl;
        return m_default_colour; 
    }

private:
    Colour light_by_Lambert(const LiteMath::float3 &to_light_dir, 
        const LiteMath::float3 &normal_dir, const Colour &object_colour) 
    {
        float lambert_coefficient = std::max(LiteMath::dot(to_light_dir, normal_dir), 0.1f);

        auto multiply_channel = [lambert_coefficient] (uint8_t channel) {
            return std::clamp<uint8_t>(channel * lambert_coefficient 
                , std::numeric_limits<uint8_t>::min()
                , std::numeric_limits<uint8_t>::max());
        };
        return {multiply_channel(object_colour.Blue)
            , multiply_channel(object_colour.Green)
            , multiply_channel(object_colour.Red)
            , object_colour.Alpha};
    }

    bool is_in_shadow(Ray ray, bool debug) {
        for (int i = 0; i < m_objects.size(); ++i) {
            IntersectionResult from_object_to_other = m_objects[i]->intersect(ray, debug);
            if (std::holds_alternative<Hit>(from_object_to_other)) {
                return true;
            }
        }
        return false;
    }

private:
    const LiteMath::float3 m_light_position;
    std::vector<std::unique_ptr<IObject>> m_objects; 
    const Colour m_default_colour = {250,206,135, 0};
    const Colour m_shadow_colour = {51, 51, 51, 0};
};

} // namespace

std::unique_ptr<IScene> create_scene(std::vector<std::unique_ptr<IObject>> objects, LiteMath::float3 light_position) 
{
    return std::make_unique<Scene>(std::move(objects), light_position);
}