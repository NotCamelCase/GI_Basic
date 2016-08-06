#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>
#undef _USE_MATH_DEFINES
#include <random>
#include <fstream>

using namespace std;

typedef unsigned char uchar;

#define WIDTH 1024
#define HEIGHT 768
#define FOV M_PI / 4
#define MAX_DEPTH 10

#define CLAMP(low, high, val) std::fmin(high, std::fmax(low, val))

std::default_random_engine rng;
std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

class Vec3
{
public:
	Vec3(double _x = 0., double _y = 0., double _z = 0.) : x(_x), y(_y), z(_z) {}
	~Vec3() {}

	Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
	void operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; }

	Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
	void operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; }

	Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }
	void operator*=(const Vec3& v) { x *= v.x; y *= v.y; z *= v.z; }
	void operator*=(const double val) { x *= val; y *= val; z *= val; }

	double dot(const Vec3& v) const { return (x * v.x + y * v.y + z * v.z); }
	Vec3 cross(const Vec3& v) const { return Vec3(y * v.z - v.y * z, v.x * z - x * v.z, x * v.y - v.x * y); }
	double length() const { return sqrt(length2()); }
	double length2() const { return (*this).dot(*this); }
	Vec3 normalize() const
	{
		Vec3 res;
		double l = length2();
		if (l > 0.0)
		{
			res = *this;
			l = 1. / l;
			res *= l;

			return res;
		}

		return res;
	}

	double x, y, z;
};

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

class Sphere
{
public:
	Sphere(const Vec3& c, double rad, Material type, const Vec3& Le = 0) : m_Le(Le), m_type(type), m_center(c), m_radius(rad) {}
	~Sphere() {}

	/* Check if ray intersects sphere */
	bool intersects(const Ray& ray, double* t)
	{
		Vec3 l = ray.m_origin - m_center;
		double a = ray.m_dir.dot(ray.m_dir);
		double b = 2 * ray.m_dir.dot(l);
		double c = l.dot(l) - (m_radius * m_radius);

		// b2 - 4ac
		double t1, t2;
		double det = b * b - 4 * a * c;
		if (det < 0) { return false; }
		else if (det == 0) { t1 = t2 = b * -0.5 * a; }
		else { double q = (b > 0) ? -0.5 * (b + sqrt(det)) : -0.5 * (b - sqrt(det)); t1 = q / a; t2 = c / q; }

		if (t1 > t2) std::swap(t1, t2);
		if (t1 < 0)
		{
			if (t2 > 0) *t = t2;
			else return false;
		}

		return true;
	}

	Vec3 getSurfaceNormal(const Vec3& v) { return (v - m_center).normalize(); }

	Vec3 m_center;
	double m_radius;

	Material m_type;
	Vec3 m_Kd;
	Vec3 m_Le;
};

#define NUM_SPHERE 3
Sphere* g_objects[NUM_SPHERE];

Vec3 g_frameBuffer[WIDTH * HEIGHT];

// Iterate all objects in the scene and find nearest object hit by the ray
Sphere* FindNearest(const Ray &ray)
{
	Sphere* obj = NULL;
	double t = INFINITY;
	for (int i = 0; i < NUM_SPHERE; i++)
	{
		Sphere* s = g_objects[i];
		double t1 = 0.;
		if (s->intersects(ray, &t1))
		{
			if (t1 < t)
			{
				obj = s;
				t = t1;
			}
		}
	}

	return obj;
}

// Compute a radiance value for an object based on its material type
Vec3 Shade(const Ray& ray, int depth)
{
	if (depth >= MAX_DEPTH) return 0;

	Sphere* obj = FindNearest(ray);
	if (obj)
		return obj->m_Kd;
	else
		return Vec3();
}

// Creata a local coordinate system oriented around surface normal of hit point on the object
void formLocalCS(const Vec3& N, Vec3& Nt, Vec3& Nb)
{
	if (std::abs(N.x) > std::abs(N.y))
		Nt = Vec3(N.z, 0., -N.x) * (1. / sqrt(N.x * N.x + N.z * N.z));
	else
		Nt = Vec3(0., -N.z, N.y) * (1. / sqrt(N.y * N.y + N.z * N.z));

	Nb = N.cross(Nb);
}

// Generate a uniformly sampled direction
Vec3 generateSampleDirOverHemisphere(double r1, double r2)
{
	double sinTheta = sqrt(1 - r1 * r1);
	double phi = 2 * M_PI * r2;
	double x = sinTheta * cos(phi);
	double z = sinTheta * sin(phi);

	return Vec3(x, r1, z);
}

// Run render loop to shade each pixel in frame buffer
void Render()
{
	double scale = tan(FOV * 0.5);
	double aspect = (double)WIDTH / HEIGHT;
	Vec3 origin;
	for (int y = 0; y < HEIGHT; y++)
	{
		for (int x = 0; x < WIDTH; x++)
		{
			double dx = (2 * (x + 0.5) / (double)WIDTH - 1) * aspect * scale;
			double dy = (1 - 2 * (y + 0.5) / (double)HEIGHT) * scale;
			Vec3 dir = Vec3(dx, dy, -1).normalize();
			Ray primRay(origin, dir);
			g_frameBuffer[x + y * WIDTH] = Shade(primRay, 0);
		}
	}

	double gamma = 1 / 2.2;
	std::ofstream ofs;
	ofs.open("render.ppm");
	ofs << "P6\n" << WIDTH << " " << HEIGHT << "\n255\n";
	for (int32_t i = 0; i < WIDTH * HEIGHT; ++i)
	{
		uchar r = (uchar)(255 * CLAMP(0., 1., pow(g_frameBuffer[i].x, gamma)));
		uchar g = (uchar)(255 * CLAMP(0., 1., pow(g_frameBuffer[i].y, gamma)));
		uchar b = (uchar)(255 * CLAMP(0., 1., pow(g_frameBuffer[i].z, gamma)));
		ofs << r << g << b;
	}
	ofs.close();
}

int main()
{
	Sphere* s1 = new Sphere(Vec3(-0.75, -.25, -2.5), .5f, Material::DIFFUSE);
	Sphere* s2 = new Sphere(Vec3(0.5, 0.12, -4.5), .6f, Material::DIFFUSE);
	Sphere* s3 = new Sphere(Vec3(0.5, -0.1, -1.75), .2f, Material::DIFFUSE);

	g_objects[0] = s1;
	g_objects[1] = s2;
	g_objects[2] = s3;

	s1->m_Kd = Vec3(0.25, 0.5, 0.75);
	s2->m_Kd = Vec3(0.35, 0.41, 0.15);
	s3->m_Kd = Vec3(0.25, 0.51, 0.23);

	Render();

	return 0;
}