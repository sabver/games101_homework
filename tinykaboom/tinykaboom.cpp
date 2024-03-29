#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

const float sphere_radius = 1.5;
const float noise_amplitude = 1.0;

template <typename T>
inline T lerp(const T &v0, const T &v1, float t)
{
    return v0 + (v1 - v0) * std::max(0.f, std::min(1.f, t));
}

float hash(const float n)
{
    float x = sin(n) * 43758.5453f;
    return x - floor(x);
}

float noise(const Vec3f &x)
{
    Vec3f p(floor(x.x), floor(x.y), floor(x.z));
    Vec3f f(x.x - p.x, x.y - p.y, x.z - p.z);
    f = f * (f * (Vec3f(3.f, 3.f, 3.f) - f * 2.f));
    float n = p * Vec3f(1.f, 57.f, 113.f);
    return lerp(lerp(
                    lerp(hash(n + 0.f), hash(n + 1.f), f.x),
                    lerp(hash(n + 57.f), hash(n + 58.f), f.x), f.y),
                lerp(
                    lerp(hash(n + 113.f), hash(n + 114.f), f.x),
                    lerp(hash(n + 170.f), hash(n + 171.f), f.x), f.y),
                f.z);
}

Vec3f rotate(const Vec3f &v)
{
    return Vec3f(Vec3f(0.00, 0.80, 0.60) * v, Vec3f(-0.80, 0.36, -0.48) * v, Vec3f(-0.60, -0.48, 0.64) * v);
}

float fractal_brownian_motion(const Vec3f &x)
{
    Vec3f p = rotate(x);
    float f = 0;
    f += 0.5000 * noise(p);
    p = p * 2.32;
    f += 0.2500 * noise(p);
    p = p * 3.03;
    f += 0.1250 * noise(p);
    p = p * 2.61;
    f += 0.0625 * noise(p);
    return f / 0.9375;
}

Vec3f palette_fire(const float d)
{
    const Vec3f yellow(1.7, 1.3, 1.0); // note that the color is "hot", i.e. has components >1
    const Vec3f orange(1.0, 0.6, 0.0);
    const Vec3f red(1.0, 0.0, 0.0);
    const Vec3f darkgray(0.2, 0.2, 0.2);
    const Vec3f gray(0.4, 0.4, 0.4);

    float x = std::max(0.f, std::min(1.f, d));
    if (x < .25f)
        return lerp(gray, darkgray, x * 4.f);
    else if (x < .5f)
        return lerp(darkgray, red, x * 4.f - 1.f);
    else if (x < .75f)
        return lerp(red, orange, x * 4.f - 2.f);
    return lerp(orange, yellow, x * 4.f - 3.f);
}

float signed_distance(const Vec3f &p)
{
    // return p.norm() - sphere_radius;
    // displacement mapping
    // Vec3f s = Vec3f(p).normalize(sphere_radius);
    // float displacement = sin(16 * s.x) * sin(16 * s.y) * sin(16 * s.z) * noise_amplitude;

    // draw disconnected components
    // float displacement = sin(16 * p.x) * sin(16 * p.y) * sin(16 * p.z) * noise_amplitude;

    float displacement = -fractal_brownian_motion(p * 3.4) * noise_amplitude;
    return p.norm() - (sphere_radius + displacement);
}
/**
 * @brief
 * 注意，这个方法的主要思想。如果在光线追踪的文章中，我们用分析法解决了光线和球体的交点，那么现在我用数值法计算它。
 * 这个想法很简单：球体有一个形式为x^2 + y^2 + z^2 - r^2 = 0的方程；但函数f(x,y,z) = x^2 + y^2 + y^2 + z^2 - r^2是在三维空间的任何地方定义。
 * 在球体内部，函数f(x,y,z)取负值，而在球体外部是正值。也就是说，函数f(x,y,z)计算每一个点(x,y,z)到我们球体的有符号距离。
 * 因此，我们将简单地沿着射线滑动，直到我们感到厌烦或者函数f(x,y,z)变成负数。这正是sphere_trace()所做的。

 * @param orig
 * @param dir
 * @param pos
 * @return true
 * @return false
 */
bool sphere_trace(const Vec3f &orig, const Vec3f &dir, Vec3f &pos)
{
    if (orig * orig - pow(orig * dir, 2) > pow(sphere_radius, 2))
        return false; // early discard

    pos = orig;
    for (size_t i = 0; i < 128; i++)
    {
        // 圆的圆心默认是在原点
        float d = signed_distance(pos);
        if (d < 0)
            return true;
        pos = pos + dir * std::max(d * 0.1f, .01f);
    }
    return false;
}
// finite difference ^f(x) = f(x+1) - f(x)
Vec3f distance_field_normal(const Vec3f &pos)
{
    const float eps = 0.1;
    float d = signed_distance(pos);
    float nx = signed_distance(pos + Vec3f(eps, 0, 0)) - d;
    float ny = signed_distance(pos + Vec3f(0, eps, 0)) - d;
    float nz = signed_distance(pos + Vec3f(0, 0, eps)) - d;
    return Vec3f(nx, ny, nz).normalize();
}

int main()
{
    const int width = 640;
    const int height = 480;
    const float fov = M_PI / 3.;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++)
    { // actual rendering loop
        for (size_t i = 0; i < width; i++)
        {
            // 镜头在(0, 0, 3)往-z看，需要注意（dir_x, dir_y， dir_z）方向的角度解释
            float dir_x = (i + 0.5) - width / 2.;
            float dir_y = -(j + 0.5) + height / 2.; // this flips the image at the same time
            float dir_z = -height / (2. * tan(fov / 2.));
            Vec3f hit;
            if (sphere_trace(Vec3f(0, 0, 3), Vec3f(dir_x, dir_y, dir_z).normalize(), hit))
            { // the camera is placed to (0,0,3) and it looks along the -z axis
                float noise_level = (sphere_radius - hit.norm()) / noise_amplitude;
                Vec3f light_dir = (Vec3f(10, 10, 10) - hit).normalize(); // one light is placed to (10,10,10)
                float light_intensity = std::max(0.4f, light_dir * distance_field_normal(hit));
                framebuffer[i + j * width] = palette_fire((-.2 + noise_level) * 2) * light_intensity;
            }
            else
            {
                framebuffer[i + j * width] = Vec3f(0.2, 0.7, 0.8); // background color
            }
        }
    }

    std::ofstream ofs("./out.ppm", std::ios::binary); // save the framebuffer to file
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        for (size_t j = 0; j < 3; j++)
        {
            ofs << (char)(std::max(0, std::min(255, static_cast<int>(255 * framebuffer[i][j]))));
        }
    }
    ofs.close();

    return 0;
}
