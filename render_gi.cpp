#include <iostream>
#include <random>
#include <chrono>
#include <vector>

#include "MathLib.h"

#undef NULL
#define NULL nullptr
#define SAFE_DELETE(x) { if (x) { delete x; x = NULL; }}

using namespace std;

typedef unsigned char uchar;
typedef unsigned int uint;

#define WIDTH 1280
#define HEIGHT 720
#define FOV M_PI / 4
#define MAX_DEPTH 10
#define SPP 256

std::default_random_engine generator;
std::uniform_real_distribution<float> distribution(0, 1);

class Ray
{
public:
	Ray(const Vec3& o = 0, const Vec3& d = 0) : m_origin(o), m_dir(d) {}
	~Ray() {}

	Vec3 m_origin;
	Vec3 m_dir;
};

enum Material
{
	DIFFUSE
};

enum class RayResult { HIT, MISS };

class Primitive
{
public:
	Primitive(const Vec3& pos, const Vec3& albedo, Material type, const Vec3& Le = 0) : m_position(pos), m_type(type), m_Kd(albedo), m_Le(Le) {}
	virtual ~Primitive() {}

	virtual RayResult intersects(const Ray& ray, float& t) const = 0;
	virtual Vec3 getSurfaceNormal(const Vec3& point = Vec3(0)) const = 0;

	Vec3 m_position;
	Material m_type;

	Vec3 m_Kd = 0.5f;
	Vec3 m_Le = 0.f;

	bool isLight() const { return length2(m_Le) > 0.f; }
};

class Sphere : public Primitive
{
public:
	Sphere(const Vec3 &cent, float rad, const Vec3& sc, Material type, const Vec3& Le = 0) : Primitive(cent, sc, type, Le), m_radius(rad) {}
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
	Plane(const Vec3 &n, float d, const Vec3& sc, Material type) : Primitive(n, sc, type), m_d(d) {}
	~Plane() {}

	virtual RayResult intersects(const Ray & ray, float & t) const override
	{
		float l = dot(ray.m_dir, m_position);
		if (!(l <= EPSILON && l > EPSILON))
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

	virtual Vec3 getSurfaceNormal(const Vec3 & point = Vec3(0)) const override
	{
		return m_position;
	}

private:
	float m_d;
};

std::vector<Primitive*> g_objects;

Vec3 g_frameBuffer[WIDTH * HEIGHT];

// Iterate all objects in the scene and find nearest object hit by the ray and assign intersection point
Primitive* FindNearest(const Ray &ray, float& _t)
{
	Primitive* obj = NULL;
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

// Creata a local coordinate system oriented around surface normal of hit point on the object
void formLocalCS(const Vec3& N, Vec3& Nt, Vec3& Nb)
{
	/*if (std::abs(N.x) > std::abs(N.y))
		Nt = Vec3(N.z, 0., -N.x) * (1. / sqrt(N.x * N.x + N.z * N.z));
	else
		Nt = Vec3(0., -N.z, N.y) * (1. / sqrt(N.y * N.y + N.z * N.z));

	Nb = cross(N, Nb);
*/
	if (std::abs(N.x) > std::abs(N.y)) {
		// project to the y = 0 plane and construct a normalized orthogonal vector in this plane
		float invLen = 1.f / sqrtf(N.x * N.x + N.z * N.z);
		Nt = Vec3(-N.z * invLen, 0.0f, N.x * invLen);
	}
	else {
		// project to the x = 0 plane and construct a normalized orthogonal vector in this plane
		float invLen = 1.0f / sqrtf(N.y * N.y + N.z * N.z);
		Nt = Vec3(0.0f, N.z * invLen, -N.y * invLen);
	}
	Nb = cross(N, Nt);
}

// Generate a uniformly sampled direction
Vec3 generateSampleDirOverHemisphere(float r1, float r2)
{
	float sinTheta = sqrt(1 - r1 * r1);
	float phi = 2 * M_PI * r2;
	float x = sinTheta * cos(phi);
	float z = sinTheta * sin(phi);

	return Vec3(x, r1, z);
}

// Compute a radiance value for an object based on its material type
Vec3 Shade(const Ray& ray, int depth)
{
	if (depth >= MAX_DEPTH)
	{
		return Vec3(0.f);
	}

	float t = 0.f;
	Primitive* obj = FindNearest(ray, t);
	if (!obj)
		return Vec3(.68f, .718f, .95f);

	Vec3 hit = ray.m_origin + ray.m_dir * t;
	Vec3 n = obj->getSurfaceNormal(hit);

#ifndef GI
	Vec3 indirectLigthing = obj->m_Le;
	Vec3 Nt, Nb;
	formLocalCS(n, Nt, Nb);
	float r1 = distribution(generator);
	float r2 = distribution(generator);
	Vec3 sample = generateSampleDirOverHemisphere(r1, r2);
	Vec3 sampleWorld(
		sample.x * Nb.x + sample.y * n.x + sample.z * Nt.x,
		sample.x * Nb.y + sample.y * n.y + sample.z * Nt.y,
		sample.x * Nb.z + sample.y * n.z + sample.z * Nt.z);
	indirectLigthing += Shade(Ray(hit, sampleWorld), depth + 1);
	indirectLigthing *= obj->m_Kd;

	return indirectLigthing;
#else
	Vec3 color = 0.f;
	for (const Primitive* primitive : g_objects)
	{
		if (primitive->isLight())
		{
			Vec3 l = normalize(primitive->m_position - hit);
			float nDotL = fmaxf(0.f, dot(n, l));
			color += obj->m_Kd * primitive->m_Le * nDotL / M_PI;
		}
	}

	return color;
#endif
}

// Run render loop to shade each pixel in frame buffer
void Render()
{
	double scale = tan(FOV * 0.5);
	double aspect = (double)WIDTH / HEIGHT;
	Vec3 origin = 0.f;
	//#pragma omp parallel for schedule(dynamic)
	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			Vec3 radiance = 0;
			for (int s = 0; s < SPP; ++s)
			{
				float dx = (2 * (x + 0.5) / (double)WIDTH - 1) * aspect * scale;
				float dy = (1 - 2 * (y + 0.5) / (double)HEIGHT) * scale;
				dx += distribution(generator) / 700;
				dy += distribution(generator) / 700;
				Vec3 dir = normalize(Vec3(dx, dy, -1));
				Ray primRay(origin, dir);
				radiance += Shade(primRay, 0);
			}

			g_frameBuffer[x + y * WIDTH] += radiance / SPP;
		}
	}
}

void OutputFrame()
{
	const double gamma = 1 / 2.2;
	FILE *f = NULL;
	fopen_s(&f, "render.ppm", "w");
	fprintf(f, "P3\n%d %d\n%d\n ", WIDTH, HEIGHT, 255);
	for (int32_t i = 0; i < WIDTH * HEIGHT; ++i)
	{
		uint r = (255 * fmin(pow(g_frameBuffer[i].x, gamma), 1.));
		uint g = (255 * fmin(pow(g_frameBuffer[i].y, gamma), 1.));
		uint b = (255 * fmin(pow(g_frameBuffer[i].z, gamma), 1.));
		fprintf(f, "%d %d %d ", r, g, b);
	}
	fclose(f);
}

int main(int argc, char* argv[])
{
	printf("Setting scene...\n");

	Sphere* s1 = new Sphere(Vec3(-0.75, -.25, -2.5), .5f, Vec3(0.25, 0.5, 0.75), Material::DIFFUSE);
	Sphere* s2 = new Sphere(Vec3(0.5, 0., -4.5), .6f, Vec3(0.13, 0.71, 0.), Material::DIFFUSE);
	Sphere* s3 = new Sphere(Vec3(0.5, -0.1, -1.75), .2f, Vec3(0.05, 0.11, 0.23), Material::DIFFUSE);

	g_objects.push_back(s1);
	g_objects.push_back(s2);
	g_objects.push_back(s3);

	g_objects.push_back(new Plane(Vec3(1, 0, 0), 1.25, Vec3(0.75, .25, .25), Material::DIFFUSE)); // Left
	g_objects.push_back(new Plane(Vec3(-1, 0, 0), 1.25, Vec3(0.25, 0.25, 0.75), Material::DIFFUSE)); // Right
	g_objects.push_back(new Plane(Vec3(0, 1, 0), .75, Vec3(0.75), Material::DIFFUSE)); // Bottom
	g_objects.push_back(new Plane(Vec3(0, -1, 0), 1.25, Vec3(0.75), Material::DIFFUSE)); // Top
	g_objects.push_back(new Plane(Vec3(0, 0, 1), 5, Vec3(0.5, 0.25, .125), Material::DIFFUSE)); // Front
	g_objects.push_back(new Plane(Vec3(0, 0, -1), 5, Vec3(0.75), Material::DIFFUSE)); // Back

	g_objects.push_back(new Sphere(Vec3(0., 2.5f, -1.5), 5.f, Vec3(1.f), Material::DIFFUSE, Vec3(1.f)));

	printf("Scene initialized\n");

	auto timer = chrono::high_resolution_clock();

	auto begin = timer.now();
	printf("Starting to render...\n");
	Render();
	auto end = timer.now();
	auto diff = chrono::duration_cast<chrono::milliseconds> (end - begin).count();
	printf("Render time: %lld ms\n", diff);

	begin = timer.now();
	OutputFrame();
	end = timer.now();
	diff = chrono::duration_cast<chrono::milliseconds> (end - begin).count();
	printf("Export time: %lld ms\n", diff);

	system("pause");

	return 0;
}