#include "object.h"
#include "mesh.h"

#include <iostream>
#include <cmath>

namespace {

class Plane 
    : public IObject
{
public:
    Plane (float a, float b, float c, float d, Colour colour, bool reflective) 
    {
        m_normal_dir = LiteMath::normalize(LiteMath::float3{a, b, c});
        m_offset = -d / LiteMath::length(LiteMath::float3{a, b, c});
        m_colour = colour;
        m_reflective = reflective;
    }

    IntersectionResult intersect(Ray ray, bool debug) override
    {

        float dividor = LiteMath::dot(ray.ray_dir, m_normal_dir);

        if (std::abs(dividor) < 1e-8f) {
            return Miss{};
        }

        float t = (m_offset - dot(ray.ray_pos, m_normal_dir)) / dividor;

        if (debug) {
            std::cout << "Coeff t: " << t << std::endl;
            std::cout << "Dividor: " << dividor << std::endl;
            std::cout << "Ray origin: " << ray.ray_pos.x << " " << ray.ray_pos.y << " " << ray.ray_pos.z << std::endl;
            std::cout << "Ray direction: " << ray.ray_dir.x << " " << ray.ray_dir.y << " " << ray.ray_dir.z << std::endl;
            std::cout << "Normal direction: " << m_normal_dir.x << " " << m_normal_dir.y << " " << m_normal_dir.z << std::endl;
            std::cout << "Plane offset: " << m_offset << std::endl;

        }

        if (t < 0.0001f || t > 100000.0f) {
            return Miss{};
        }

        Hit result = {};
        result.t = t;
        result.normal_dir = m_normal_dir;
        result.hit_position = ray.ray_pos + result.t * ray.ray_dir;
        result.colour = m_colour;
        result.is_reflective = m_reflective;
        return result;
    }

private:
    LiteMath::float3 m_normal_dir;
    float m_offset; // plane <=> dot(normal, point) + offset = 0
    bool m_reflective = false;
    Colour m_colour;
};

class Triangle 
    : public IObject
{
public:
    Triangle(
        LiteMath::float3 A_pos,
        LiteMath::float3 B_pos,
        LiteMath::float3 C_pos,
        LiteMath::float3 A_norm_dir,
        LiteMath::float3 B_norm_dir,
        LiteMath::float3 C_norm_dir) 
            : m_A_pos(A_pos)
            , m_B_pos(B_pos)
            , m_C_pos(C_pos)
            , m_A_norm_dir(A_norm_dir)
            , m_B_norm_dir(B_norm_dir)
            , m_C_norm_dir(C_norm_dir)
    { 
        m_centroid = (m_A_pos + m_B_pos + m_C_pos) / 3.0f;
    }

    IntersectionResult intersect(Ray ray, bool debug) override
    {
        /*LiteMath::float3x3 M = LiteMath::make_float3x3_by_columns(
                m_A_pos - ray.ray_pos, 
                m_B_pos - ray.ray_pos,
                m_C_pos - ray.ray_pos);
        LiteMath::float3 coefficients = LiteMath::inverse3x3(M) * ray.ray_dir;
        if (debug) {
            std::cout << "Coeff: " << coefficients.x << " " <<  coefficients.y << " " << coefficients.z <<std::endl;
            std::cout << "Ray origin: " << ray.ray_pos.x << " " << ray.ray_pos.y << " " << ray.ray_pos.z << std::endl;
            std::cout << "Ray direction: " << ray.ray_dir.x << " " << ray.ray_dir.y << " " << ray.ray_dir.z << std::endl;
        }

        if (coefficients.x >= 1e-8f && coefficients.y >= 1e-8f && coefficients.z >= 1e-8f) {
            float t = 1 / (coefficients.x + coefficients.y + coefficients.z);
            if (t < 0.0001f || t > 100000.0f) {
                return Miss{};
            }
            Hit result = {};
            result.t = t;
            result.hit_position = ray.ray_pos + result.t * ray.ray_dir;
            result.normal_dir = get_norm(result.hit_position);
            result.colour = {0,255,0,0};
            return result;
        }
        return Miss{}; */


        constexpr float epsilon = std::numeric_limits<float>::epsilon();

        LiteMath::float3 edge1 = m_B_pos - m_A_pos;
        LiteMath::float3 edge2 = m_C_pos - m_A_pos;
        LiteMath::float3 ray_cross_e2 = cross(ray.ray_dir, edge2);
        float det = dot(edge1, ray_cross_e2);

        if (det > -epsilon && det < epsilon)
            return Miss{};    // This ray is parallel to this triangle.

        float inv_det = 1.0 / det;
        LiteMath::float3 s = ray.ray_pos - m_A_pos;
        float u = inv_det * dot(s, ray_cross_e2);

        if ((u < 0 && fabs(u) > epsilon) || (u > 1 && fabs(u - 1) > epsilon))
            return Miss{};

        LiteMath::float3 s_cross_e1 = cross(s, edge1);
        float v = inv_det * dot(ray.ray_dir, s_cross_e1);

        if ((v < 0 && fabs(v) > epsilon) || (u + v > 1 && fabs(u + v - 1) > epsilon))
            return Miss{};

        // At this stage we can compute t to find out where the intersection point is on the line.
        float t = inv_det * dot(edge2, s_cross_e1);

        if (t > epsilon) // ray intersection
        {
            Hit result = {};
            result.t = t;
            result.hit_position = ray.ray_pos + result.t * ray.ray_dir;
            result.normal_dir = get_norm(result.hit_position);
            result.colour = {0,255,0,0};
            return result;
        }
        else // This means that there is a line intersection but not a ray intersection.
            return Miss{};
    }

    LiteMath::float3 get_A_pos() {
        return m_A_pos;
    }   

    LiteMath::float3 get_B_pos() {
        return m_B_pos;
    } 

    LiteMath::float3 get_C_pos() {
        return m_C_pos;
    }  

    LiteMath::float3 get_centroid() {
        return m_centroid;
    } 

private:
    LiteMath::float3 get_norm(LiteMath::float3 dot_pos) {
        float A_abs = LiteMath::length(dot_pos - m_A_pos);
        float B_abs = LiteMath::length(dot_pos - m_B_pos);
        float C_abs = LiteMath::length(dot_pos - m_C_pos);
        return (m_A_norm_dir * A_abs + m_B_norm_dir * B_abs + m_C_norm_dir * C_abs) / (A_abs + B_abs + C_abs);
    }

private:
    LiteMath::float3 m_A_pos;
    LiteMath::float3 m_B_pos;
    LiteMath::float3 m_C_pos;
    LiteMath::float3 m_A_norm_dir;
    LiteMath::float3 m_B_norm_dir;
    LiteMath::float3 m_C_norm_dir;
    LiteMath::float3 m_centroid;
};

struct BVHNode {
    LiteMath::float3 aa_bounding_box_max;
    LiteMath::float3 aa_bounding_box_min;
    uint32_t left, right;
    uint32_t first_primitive_idx, primitives_count;
    bool is_leaf() {
        return primitives_count > 0;
    }
};

class BVH {
public:

    BVH ()
    { }

    explicit BVH (const std::vector<Triangle>& triangles) 
        : m_triangles(&triangles)
        , m_bvh_nodes(triangles.size() * 2 - 1)
    {
        for (int i = 0; i < triangles.size(); ++i) {
            m_triangles_idx.push_back(i);
        }
        BVHNode& root = m_bvh_nodes[m_root_node_idx];
        root.left = root.right = 0;
        root.first_primitive_idx = 0;
        root.primitives_count = triangles.size();
        update_node_bounds(m_root_node_idx);
        subdivide(m_root_node_idx);
    }

    void update_node_bounds(uint32_t node_idx)
    {
        BVHNode& node = m_bvh_nodes[node_idx];
        node.aa_bounding_box_min = LiteMath::float3(
            std::numeric_limits<float>::max(), 
            std::numeric_limits<float>::max(), 
            std::numeric_limits<float>::max());
        node.aa_bounding_box_max = LiteMath::float3(
            std::numeric_limits<float>::min(), 
            std::numeric_limits<float>::min(), 
            std::numeric_limits<float>::min());

        for (uint32_t first = node.first_primitive_idx, i = 0; i < node.primitives_count; ++i)
        {
            uint32_t leaf_triangle_idx = m_triangles_idx[first + i];
            Triangle& leaf_triangle = (*m_triangles)[leaf_triangle_idx];

            node.aa_bounding_box_min = LiteMath::min(node.aa_bounding_box_min, leaf_triangle.get_A_pos());
            node.aa_bounding_box_min = LiteMath::min(node.aa_bounding_box_min, leaf_triangle.get_B_pos());
            node.aa_bounding_box_min = LiteMath::min(node.aa_bounding_box_min, leaf_triangle.get_C_pos());
            node.aa_bounding_box_max = LiteMath::max(node.aa_bounding_box_max, leaf_triangle.get_A_pos());
            node.aa_bounding_box_max = LiteMath::max(node.aa_bounding_box_max, leaf_triangle.get_B_pos());
            node.aa_bounding_box_max = LiteMath::max(node.aa_bounding_box_max, leaf_triangle.get_C_pos());
        }
    }

    void subdivide(uint32_t node_idx)
    {
        BVHNode& node = m_bvh_nodes[node_idx];
        if (node.primitives_count <= 2) {
            return;
        }

        LiteMath::float3 extent = node.aa_bounding_box_max - node.aa_bounding_box_min;
        int axis = 0;
        if (extent.y > extent.x) {
            axis = 1;
        }
        if (extent.z > extent[axis]) {
            axis = 2;
        }
        float splitPos = node.aa_bounding_box_min[axis] + extent[axis] * 0.5f;


        int i = node.first_primitive_idx;
        int j = i + node.primitives_count - 1;
        while (i <= j)
        {
            if (m_triangles[m_triangles_idx[i]].get_centroid()[axis] < splitPos) {
                ++i;
            }
            else {
                std::swap(m_triangles_idx[i], m_triangles_idx[j]);
                --j;
            }
        }

        int left_count = i - node.first_primitive_idx;
        if (left_count == 0 || left_count == node.primitives_count) {
            return;
        }
        
        int left_child_idx = m_node_used;
        ++m_node_used;
        int right_child_idx = m_node_used;
        ++m_node_used;
        //node.left = left_child_idx;
        m_bvh_nodes[left_child_idx].first_primitive_idx = node.first_primitive_idx;
        m_bvh_nodes[left_child_idx].primitives_count = left_count;
        m_bvh_nodes[right_child_idx].first_primitive_idx = i;
        m_bvh_nodes[right_child_idx].primitives_count = node.primitives_count - left_count;
        node.left = left_child_idx;
        node.right = right_child_idx;
        node.primitives_count = 0;
        update_node_bounds(left_child_idx);
        update_node_bounds(right_child_idx);
        subdivide(left_child_idx);
        subdivide(right_child_idx);
    }

    bool is_intersected_AABB (Ray& ray, LiteMath::float3 aabb_min, LiteMath::float3 aabb_max) {
        float tx1 = (aabb_min.x - ray.ray_pos.x) / ray.ray_dir.x;
        float tx2 = (aabb_max.x - ray.ray_pos.x) / ray.ray_dir.x;
        float tmin = std::min(tx1, tx2);
        float tmax = std::max(tx1, tx2);

        float ty1 = (aabb_min.y - ray.ray_pos.y) / ray.ray_dir.y;
        float ty2 = (aabb_max.y - ray.ray_pos.y) / ray.ray_dir.y;
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));


        float tz1 = (aabb_min.z - ray.ray_pos.z) / ray.ray_dir.z;
        float tz2 = (aabb_max.z - ray.ray_pos.z) / ray.ray_dir.z;
        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));
        return tmax >= tmin && tmin < 1e30f && tmax > 0;

    }

    IntersectionResult intersect_BVHNode (Ray& ray, const uint32_t node_idx, bool debug) {
        Hit closest;
        closest.t = std::numeric_limits<float>::max();

        BVHNode& node = m_bvh_nodes[node_idx];
        if (!is_intersected_AABB(ray, node.aa_bounding_box_min, node.aa_bounding_box_max)) {
            return Miss{};
        }
        if (node.is_leaf()) {
            for (uint i = 0; i < node.primitives_count; ++i) {
                IntersectionResult current = m_triangles[m_triangles_idx[node.first_primitive_idx + i]].intersect(ray, debug);
                if (auto* hit = std::get_if<Hit>(&current); hit && hit->t < closest.t) {
                    closest = *hit;
                }
            }
        }
        else {
            IntersectionResult left_closest = intersect_BVHNode(ray, node.left, debug);
            if (auto* hit = std::get_if<Hit>(&left_closest); hit && hit->t < closest.t) {
                closest = *hit;
            }
            IntersectionResult right_closest = intersect_BVHNode(ray, node.right, debug);
            if (auto* hit = std::get_if<Hit>(&right_closest); hit && hit->t < closest.t) {
                closest = *hit;
            }
        }
        if (closest.t == std::numeric_limits<float>::max()) {
            return Miss{};
        }
        return closest;
    }

private:
    std::vector<Triangle>* m_triangles;
    std::vector<uint32_t> m_triangles_idx;
    std::vector<BVHNode> m_bvh_nodes;
    uint32_t m_root_node_idx = 0;
    uint32_t m_node_used = 1;
};


class Mesh
    : public IObject
{
public:
    Mesh(const char* file_name, Colour colour, bool reflective) 
    {
        cmesh4::SimpleMesh mesh = cmesh4::LoadMeshFromObj(file_name);
        m_colour = colour;
        m_reflective = reflective;
        for (auto i = 0; i < mesh.TrianglesNum(); ++i) {
            int vertA_idx = mesh.indices[i * 3];
            int vertB_idx = mesh.indices[i * 3 + 1];
            int vertC_idx = mesh.indices[i * 3 + 2];
            m_triangles.push_back(Triangle(
                {mesh.vPos4f[vertA_idx].x, mesh.vPos4f[vertA_idx].y, mesh.vPos4f[vertA_idx].z},
                {mesh.vPos4f[vertB_idx].x, mesh.vPos4f[vertB_idx].y, mesh.vPos4f[vertB_idx].z},
                {mesh.vPos4f[vertC_idx].x, mesh.vPos4f[vertC_idx].y, mesh.vPos4f[vertC_idx].z},
                {mesh.vNorm4f[vertA_idx].x, mesh.vNorm4f[vertA_idx].y, mesh.vNorm4f[vertA_idx].z},
                {mesh.vNorm4f[vertB_idx].x, mesh.vNorm4f[vertB_idx].y, mesh.vNorm4f[vertB_idx].z},
                {mesh.vNorm4f[vertC_idx].x, mesh.vNorm4f[vertC_idx].y, mesh.vNorm4f[vertC_idx].z}             
            ));
        }
        m_bvh = BVH(m_triangles);
    }

    IntersectionResult intersect(Ray ray, bool debug) override
    {
        return intersect_BVH(ray, debug);
    }

    IntersectionResult intersect_simple(Ray ray, bool debug)
    {
        Hit closest;
        closest.t = std::numeric_limits<float>::max(); 
        for (auto triangle : m_triangles) {
            
            auto current = triangle.intersect(ray, debug);
    
            if (std::holds_alternative<Hit>(current) 
                && std::get<Hit>(current).t < closest.t) {
                closest = std::get<Hit>(current);
            }
        }
        if (closest.t == std::numeric_limits<float>::max()) {
            return Miss{};
        }
        closest.colour = m_colour;
        closest.is_reflective = m_reflective;
        return closest;
    }

    IntersectionResult intersect_BVH(Ray ray, bool debug)
    {
        IntersectionResult result = m_bvh.intersect_BVHNode(ray, 0, debug);
        if (auto* hit = std::get_if<Hit>(&result); hit) {
            hit->colour = m_colour;
            hit->is_reflective = m_reflective;
        }
        return result;
    }


private:
    Colour m_colour;
    bool m_reflective;
    std::vector<Triangle> m_triangles;
    BVH m_bvh;
};

} // namespace

std::unique_ptr<IObject> create_object_plane(float a, float b, float c, float d, Colour colour, bool reflective) 
{
    return std::make_unique<Plane>(a, b, c, d, colour, reflective);
}

std::unique_ptr<IObject> create_object_mesh(const char* file_name, Colour colour, bool reflective) 
{
    return std::make_unique<Mesh>(file_name, colour, reflective);
}

std::unique_ptr<IObject> create_object_triangle(
    LiteMath::float3 A_pos,
    LiteMath::float3 B_pos,
    LiteMath::float3 C_pos,
    LiteMath::float3 A_norm_dir,
    LiteMath::float3 B_norm_dir,
    LiteMath::float3 C_norm_dir) 
{
    return std::make_unique<Triangle>(
        A_pos,
        B_pos,
        C_pos,
        A_norm_dir,
        B_norm_dir,
        C_norm_dir);
}