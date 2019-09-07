#include <iostream>
#include <cstdint>
#include <random>
#include <chrono>
#include <vector>

#include "MathLib.h"

using namespace std;

#define WIDTH 1920
#define HEIGHT 1080
#define FOV (M_PI / 3.5)
#define MAX_DEPTH 10
#define NUM_SAMPLES (64 * 64)

static default_random_engine g_rndEngine;
static uniform_real_distribution<float> g_uniformRnd(0.f, 1.f);

class Ray
{
public:
    Ray(const Vec3& o = 0, const Vec3 & d = 0) : m_origin(o), m_dir(d) {}
    ~Ray() {}

    Vec3 m_origin;
    Vec3 m_dir;
};

enum class Material
{
    DIFFUSE,
    SPECULAR,
    REFRACT
};

enum class RayResult { HIT, MISS };

class Primitive
{
public:
    Primitive(const Vec3& pos, const Vec3& albedo, Material type, const Vec3& Le = 0, float ior = 0.f) : m_ior(ior), m_position(pos), m_type(type), m_Kd(albedo), m_Le(Le) {}
    virtual ~Primitive() {}

    virtual RayResult intersects(const Ray& ray, float& t) const = 0;
    virtual Vec3 getSurfaceNormal(const Vec3& point = Vec3(0)) const = 0;

    Vec3 m_position;
    Material m_type;

    Vec3 m_Kd = 0.5f;
    Vec3 m_Le = 0.f;
    float m_ior = 1.f;

    bool isLight() const { return length2(m_Le) > 0.f; }
};

class Sphere : public Primitive
{
public:
    Sphere(const Vec3& cent, float rad, const Vec3& sc, Material type, const Vec3& Le = 0, float ior = 0.f) : Primitive(cent, sc, type, Le, ior), m_radius(rad) {}
    ~Sphere() {}

    virtual RayResult intersects(const Ray& ray, float& t) const override
    {
        Vec3 v = m_position - ray.m_origin;
        float b = dot(v, ray.m_dir);
        if (b < 0) return RayResult::MISS;

        float d2 = dot(v, v) - b * b;
        if (d2 > m_radius * m_radius) return RayResult::MISS;

        float det = sqrt(m_radius * m_radius - d2);
        float t0 = b - det;
        float t1 = b + det;

        if (t0 < 0) t0 = t1;
        t = t0;

        return RayResult::HIT;
    }

    virtual Vec3 getSurfaceNormal(const Vec3& hit = Vec3(0)) const override
    {
        return normalize(hit - m_position);
    }

    float m_radius;
};

class Plane : public Primitive
{
public:
    Plane(const Vec3& n, float d, const Vec3& sc, Material type, float ior = 0.f) : Primitive(n, sc, type, 0.f, ior), m_d(d) {}
    ~Plane() {}

    virtual RayResult intersects(const Ray& ray, float& t) const override
    {
        float l = dot(ray.m_dir, m_position);
        if (fabs(l) > EPSILON)
        {
            float dist = -(dot(m_position, ray.m_origin) + m_d) / l;
            if (dist > 0)
            {
                t = dist;

                return RayResult::HIT;
            }
        }

        return RayResult::MISS;
    }

    virtual Vec3 getSurfaceNormal(const Vec3& point = Vec3(0)) const override
    {
        return m_position;
    }

private:
    float m_d;
};

vector<Primitive*> g_objects;

Vec3 g_frameBuffer[WIDTH * HEIGHT];

// Iterate all objects in the scene and find nearest object hit by the ray and assign intersection point
Primitive* FindNearest(const Ray& ray, float& _t)
{
    Primitive* obj = nullptr;
    float t = INFINITY;
    for (int i = 0; i < g_objects.size(); i++)
    {
        Primitive* s = g_objects[i];
        float t1 = 0.;
        if (s->intersects(ray, t1) == RayResult::HIT)
        {
            if (t1 < t)
            {
                obj = s;
                t = t1;
            }
        }
    }

    _t = t;
    return obj;
}

// Creata a local cartesian system oriented around surface normal of hit point
void formLocalCS(const Vec3& N, Vec3& Nt, Vec3& Nb)
{
    if (fabs(N.x) > fabs(N.y))
    {
        float invLen = 1.f / sqrtf(N.x * N.x + N.z * N.z);
        Nt = Vec3(-N.z * invLen, 0.0f, N.x * invLen);
    }
    else
    {
        float invLen = 1.0f / sqrtf(N.y * N.y + N.z * N.z);
        Nt = Vec3(0.0f, N.z * invLen, -N.y * invLen);
    }

    Nb = cross(N, Nt);
}

// Generate a uniformly sampled direction
Vec3 generateSampleDirOverHemisphere(float r1, float r2)
{
    const float sint = sqrt(1 - r1 * r1);
    const float phi = 2 * M_PI * r2;

    return Vec3(sint * cos(phi), r1, sint * sin(phi));
}

// Compute a radiance value for an object based on its material type
Vec3 Shade(const Ray& ray, int depth)
{
    if (depth >= MAX_DEPTH) return Vec3(0.f);

    float t = 0.f;
    Primitive* obj = FindNearest(ray, t);
    if (!obj) return Vec3(0.5f);

    Vec3 hit = ray.m_origin + ray.m_dir * t;
    Vec3 n = obj->getSurfaceNormal(hit);

    if (obj->m_type == Material::DIFFUSE)
    {
        Vec3 Nt, Nb;
        formLocalCS(n, Nt, Nb);
        float r1 = g_uniformRnd(g_rndEngine);
        float r2 = g_uniformRnd(g_rndEngine);
        Vec3 uniSample = generateSampleDirOverHemisphere(r1, r2);
        Vec3 sampleProjected
        (
            dot(uniSample, Vec3(Nb.x, n.x, Nt.x)),
            dot(uniSample, Vec3(Nb.y, n.y, Nt.y)),
            dot(uniSample, Vec3(Nb.z, n.z, Nt.z))
        );

        return obj->m_Le + Shade(Ray(hit + sampleProjected * 1e-5, sampleProjected), depth + 1) * obj->m_Kd;
    }
    else if (obj->m_type == Material::SPECULAR)
    {
        Vec3 refdir = reflect(ray.m_dir, n);
        return Shade(Ray(hit + refdir * 1e-5, refdir), depth + 1) * obj->m_Kd;
    }
    else if (obj->m_type == Material::REFRACT)
    {
        float m_ior = obj->m_ior;
        float R0 = (1.0f - m_ior) / (1.0 + m_ior);
        R0 = R0 * R0;
        n = (dot(n, ray.m_dir) > 0 ? -n : n);
        m_ior = 1 / m_ior;
        float cost1 = -(dot(n, ray.m_dir));
        float cost2 = 1.0f - m_ior * m_ior * (1.0f - cost1 * cost1);
        Vec3 refrDir = 0.f;
        if (cost2 > 0)
        {
            refrDir = normalize((ray.m_dir * m_ior) + (n * (m_ior * cost1 - sqrtf(cost2))));
        }
        else
        {
            refrDir = normalize(ray.m_dir + n * (cost1 * 2));
        }

        return Shade(Ray(hit + refrDir * 1e-5, refrDir), depth + 1);
    }
}

// Run render loop to shade each pixel in frame buffer
void Render()
{
    double scale = tan(FOV * 0.5);
    double aspect = (double)WIDTH / HEIGHT;
    Vec3 origin = 0.f;
#ifndef DEBUG
#pragma omp parallel for schedule(dynamic)
#endif
    for (int y = 0; y < HEIGHT; y++)
    {
        fprintf(stdout, "\rRendering: %8.2f%%", (double)y / HEIGHT * 100);
        for (int x = 0; x < WIDTH; x++)
        {
            Vec3 radiance = 0.f;
            for (int i = 0; i < NUM_SAMPLES; i++)
            {
                float dx = (2 * (x + 0.5) / (double)WIDTH - 1) * aspect * scale;
                float dy = (1 - 2 * (y + 0.5) / (double)HEIGHT) * scale;
                Vec3 dir = normalize(Vec3(dx + g_uniformRnd(g_rndEngine) / 1000, dy + g_uniformRnd(g_rndEngine) / 1000, -1));
                Ray primRay(origin, dir);
                radiance += Shade(primRay, 0);
            }

            g_frameBuffer[x + y * WIDTH] = radiance / NUM_SAMPLES;
        }
    }
}

void OutputFrame()
{
    const double gamma = 1 / 2.2;
    FILE* f = nullptr;
    fopen_s(&f, "render.ppm", "w");
    fprintf_s(f, "P3\n%d %d\n%d\n ", WIDTH, HEIGHT, 255);
    for (auto i = 0; i < WIDTH * HEIGHT; ++i)
    {
        uint8_t r = (255 * fminf(pow(g_frameBuffer[i].x, gamma), 1.));
        uint8_t g = (255 * fminf(pow(g_frameBuffer[i].y, gamma), 1.));
        uint8_t b = (255 * fminf(pow(g_frameBuffer[i].z, gamma), 1.));
        fprintf_s(f, "%d %d %d ", r, g, b);
    }
    fclose(f);
}

int main(int argc, char* argv[])
{
    printf("Setting scene...\n");

    Sphere* s1 = new Sphere(Vec3(-0.80, -.25, -2.5), .5f, Vec3(1.0f, 0.f, 0.f), Material::DIFFUSE);
    Sphere* s2 = new Sphere(Vec3(1.95, 0., -4.f), .6f, Vec3(.25f, .5f, 0.025f), Material::SPECULAR);
    Sphere* s3 = new Sphere(Vec3(0.35, -0.495, -1.75), .29f, Vec3(1.f, 1.f, 0.f), Material::REFRACT, 1.55f);

    Sphere* s4 = new Sphere(Vec3(0.250, -0.125, -3.55), .35f, Vec3(0.f, 0.f, 1.f), Material::DIFFUSE);
    Sphere* s5 = new Sphere(Vec3(2.75f, 0.75f, -7.5), 1.09f, Vec3(.625f, .325f, 0.125f), Material::SPECULAR);

    g_objects.push_back(s1);
    g_objects.push_back(s2);
    g_objects.push_back(s3);
    g_objects.push_back(s4);
    g_objects.push_back(s5);

    g_objects.push_back(new Plane(Vec3(1, 0, 0), 1.28f, Vec3(0.75, .25, .25), Material::DIFFUSE)); // Left
    g_objects.push_back(new Plane(Vec3(0, 1, 0), .75, Vec3(0.5f), Material::DIFFUSE)); // Bottom

    g_objects.push_back(new Sphere(Vec3(0., 1.88f, -5), .0035f, Vec3(1.f), Material::DIFFUSE, Vec3(1.f))); // Single light

    printf("Scene initialized\n");

    auto timer = chrono::high_resolution_clock();

    auto begin = timer.now();
    Render();
    auto end = timer.now();
    auto diff = chrono::duration_cast<chrono::seconds> (end - begin).count();
    printf("\nRender time: %lld sec\n", diff);

    begin = timer.now();
    OutputFrame();
    end = timer.now();
    diff = chrono::duration_cast<chrono::seconds> (end - begin).count();
    printf("Export time: %lld sec\n", diff);

    for (Primitive* obj : g_objects)
    {
        delete obj;
    }

    system("pause");

    return 0;
}