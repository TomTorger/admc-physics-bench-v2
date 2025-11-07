#pragma once

#include "admc/scene/scene_desc.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace admc::scene
{
inline SceneDesc make_two_spheres_scene(float separation = 0.1f)
{
    SceneDesc scene{};
    scene.name = "two_spheres";

    const float radius = 0.5f;
    const float mass = 1.0f;

    BodyDesc a{};
    a.position = ::admc::core::Vec3{-radius - separation * 0.5f, radius, 0.0f};
    a.linear_velocity = ::admc::core::Vec3{1.0f, 0.0f, 0.0f};
    a.mass = mass;

    BodyDesc b{};
    b.position = ::admc::core::Vec3{radius + separation * 0.5f, radius, 0.0f};
    b.linear_velocity = ::admc::core::Vec3{-1.0f, 0.0f, 0.0f};
    b.mass = mass;

    scene.bodies.push_back(a);
    scene.bodies.push_back(b);

    ContactDesc contact{};
    contact.body_a = 0;
    contact.body_b = 1;
    contact.point = ::admc::core::Vec3{0.0f, radius, 0.0f};
    contact.normal = ::admc::core::Vec3{-1.0f, 0.0f, 0.0f};
    contact.penetration = separation;
    contact.restitution = 0.0f;

    scene.contacts.push_back(contact);
    return scene;
}

inline SceneDesc make_box_stack_scene(int height = 3)
{
    SceneDesc scene{};
    scene.name = "box_stack_" + std::to_string(height);

    for (int i = 0; i < height; ++i)
    {
        BodyDesc body{};
        body.position = ::admc::core::Vec3{0.0f, static_cast<float>(i) + 0.5f, 0.0f};
        body.linear_velocity = ::admc::core::Vec3{0.0f, 0.0f, 0.0f};
        body.mass = 1.0f;
        scene.bodies.push_back(body);
    }

    BodyDesc ground{};
    ground.position = ::admc::core::Vec3{0.0f, 0.0f, 0.0f};
    ground.mass = 0.0f;
    scene.bodies.push_back(ground);

    const int ground_id = static_cast<int>(scene.bodies.size()) - 1;
    for (int i = 0; i < height; ++i)
    {
        ContactDesc contact{};
        contact.body_a = i;
        contact.body_b = (i == 0) ? ground_id : i - 1;
        contact.point = ::admc::core::Vec3{0.0f, static_cast<float>(i), 0.0f};
        contact.normal = (i == 0) ? ::admc::core::Vec3{0.0f, -1.0f, 0.0f} : ::admc::core::Vec3{0.0f, 1.0f, 0.0f};
        contact.penetration = 0.01f;
        contact.restitution = 0.0f;
        scene.contacts.push_back(contact);
    }

    return scene;
}

inline SceneDesc make_sphere_grid_scene(const std::string& name, int rows, int cols, float spacing, float base_height = 0.5f)
{
    SceneDesc scene{};
    scene.name = name;
    const float radius = 0.5f;
    const float mass = 1.0f;

    BodyDesc ground{};
    ground.position = ::admc::core::Vec3{0.0f, 0.0f, 0.0f};
    ground.mass = 0.0f;
    scene.bodies.push_back(ground);
    const int ground_id = 0;

    const int sphere_count = rows * cols;
    std::vector<int> body_ids(sphere_count, -1);
    std::vector<::admc::core::Vec3> centers(sphere_count);

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            const int idx = r * cols + c;
            BodyDesc body{};
            body.position = ::admc::core::Vec3{
                (c - cols * 0.5f + 0.5f) * spacing,
                base_height,
                (r - rows * 0.5f + 0.5f) * spacing};
            body.linear_velocity = ::admc::core::Vec3{0.0f, (c % 2 == 0) ? 0.0f : -0.1f, 0.0f};
            body.mass = mass;
            const int body_id = static_cast<int>(scene.bodies.size());
            scene.bodies.push_back(body);
            body_ids[idx] = body_id;
            centers[idx] = body.position;

            ContactDesc ground_contact{};
            ground_contact.body_a = body_id;
            ground_contact.body_b = ground_id;
            ground_contact.point = body.position + ::admc::core::Vec3{0.0f, -radius, 0.0f};
            ground_contact.normal = ::admc::core::Vec3{0.0f, -1.0f, 0.0f};
            ground_contact.penetration = std::max(0.0f, radius - (body.position.y - radius));
            scene.contacts.push_back(ground_contact);
        }
    }

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            const int idx = r * cols + c;
            const int body_id = body_ids[idx];
            const ::admc::core::Vec3 pos_a = centers[idx];

            auto emit_contact = [&](int neighbor_idx) {
                const int neighbor_id = body_ids[neighbor_idx];
                const ::admc::core::Vec3 pos_b = centers[neighbor_idx];
                const ::admc::core::Vec3 diff = pos_b - pos_a;
                const float dist = std::sqrt(std::max(1e-6f, ::admc::core::dot(diff, diff)));
                ContactDesc contact{};
                contact.body_a = body_id;
                contact.body_b = neighbor_id;
                contact.point = pos_a + diff * 0.5f;
                contact.normal = diff / dist;
                contact.penetration = std::max(0.0f, 2.0f * radius - dist);
                scene.contacts.push_back(contact);
            };

            if (c + 1 < cols)
            {
                emit_contact(r * cols + (c + 1));
            }
            if (r + 1 < rows)
            {
                emit_contact((r + 1) * cols + c);
            }
        }
    }

    return scene;
}

inline std::vector<SceneDesc> make_default_scene_library()
{
    std::vector<SceneDesc> scenes;
    scenes.push_back(make_two_spheres_scene());
    scenes.push_back(make_box_stack_scene(3));
    scenes.push_back(make_sphere_grid_scene("sphere_grid_4x4", 4, 4, 1.05f));
    scenes.push_back(make_sphere_grid_scene("sphere_grid_8x8", 8, 8, 1.05f));
    scenes.push_back(make_sphere_grid_scene("sphere_cloud_16x16", 16, 16, 1.02f));
    scenes.push_back(make_sphere_grid_scene("sphere_cloud_32x32", 32, 32, 1.02f));
    scenes.push_back(make_sphere_grid_scene("sphere_cloud_64x64", 64, 64, 1.02f));
    return scenes;
}
} // namespace admc::scene
