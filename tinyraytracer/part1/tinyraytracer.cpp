#include <limits>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include "geometry.h"

struct Light
{
    Light(const Vec3f &p, const float &i) : position(p), intensity(i) {}
    Vec3f position;
    float intensity;
};

struct Material
{
    Material(const float &r, const Vec4f &a, const Vec3f &color, const float &spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
    // albedo[2]是镜子材质的影响因子
    Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
    float refractive_index;
    Vec4f albedo;
    Vec3f diffuse_color;
    float specular_exponent;
};

struct Sphere
{
    Vec3f center;
    float radius;
    Material material;

    Sphere(const Vec3f &c, const float &r, const Material &m) : center(c), radius(r), material(m) {}

    bool ray_intersect(const Vec3f &orig, const Vec3f &dir, float &t0) const
    {
        Vec3f L = center - orig;
        // 假设A是斜边，B是底边
        // cos<A, B> = A.B / |A|*|B|
        // A点在B边上的投影 projection<A, B> = cos<A, B> * |A| = A.B / |B|
        // 因为|B| = 1, 所以projection<A, B> = A.B
        float tca = L * dir;
        // d2是center在ray上面的投影下来的line长度
        float d2 = L * L - tca * tca;
        if (d2 > radius * radius)
            return false;
        // 也就是上面例子中的B的长度，三角形是圆内的
        float thc = sqrtf(radius * radius - d2);
        t0 = tca - thc;
        float t1 = tca + thc;
        // 判断orig是在园内还是园外
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        return true;
    }
};

Vec3f reflect(const Vec3f &I, const Vec3f &N)
{
    return I - N * 2.f * (I * N);
}

Vec3f refract(const Vec3f &I, const Vec3f &N, const float &refractive_index)
{ // Snell's law
    float cosi = -std::max(-1.f, std::min(1.f, I * N));
    // 折射率
    float etai = 1, etat = refractive_index;
    Vec3f n = N;
    if (cosi < 0)
    { // if the ray is inside the object, swap the indices and invert the normal to get the correct result
        cosi = -cosi;
        std::swap(etai, etat);
        n = -N;
    }
    // n2sin@2 = n1sin@1
    float eta = etai / etat;
    // cos？
    float k = 1 - eta * eta * (1 - cosi * cosi);
    // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
    // k<0=全反射，没有光线可以折射。无论如何我都会把它折射出去，这没有任何物理意义。
    return k < 0 ? Vec3f(0, 0, 0) : I * eta + n * (eta * cosi - sqrtf(k));
}

bool scene_intersect(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, Vec3f &hit, Vec3f &N, Material &material)
{
    float spheres_dist = std::numeric_limits<float>::max();
    for (size_t i = 0; i < spheres.size(); i++)
    {
        float dist_i;
        // dist_i是相遇的时间t，t最小那个对应的圆就是需要展示的颜色
        if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist)
        {
            spheres_dist = dist_i;
            // 交集点
            hit = orig + dir * dist_i;
            // 从圆心指向交集点
            N = (hit - spheres[i].center).normalize();
            material = spheres[i].material;
        }
    }
    return spheres_dist < 1000;
}

Vec3f cast_ray(const Vec3f &orig, const Vec3f &dir, const std::vector<Sphere> &spheres, const std::vector<Light> &lights, size_t depth = 0)
{
    // point is hit point
    // N = (hit - spheres[i].center).normalize();
    Vec3f point, N;
    Material material;

    if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material))
    {
        return Vec3f(0.2, 0.7, 0.8); // background color
    }

    Vec3f reflect_dir = reflect(dir, N).normalize();
    // offset the original point to avoid occlusion by the object itself
    // 偏移原点，以避免被物体本身遮挡
    // 这里如果直接reflect_orig = point的话，镜面上会出现很多黑点，why？
    Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);

    Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
    Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
    Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (size_t i = 0; i < lights.size(); i++)
    {
        Vec3f light_dir = (lights[i].position - point).normalize();

        // 在绘制每个点时，我们只需确保当前点和光源之间的线段不与场景中的物体相交。如果有相交，我们就跳过当前的光源。只有一个小的微妙之处：我通过在法线方向上移动点来扰动它。
        float light_distance = (lights[i].position - point).norm();
        // 为什么会这样呢？只是我们的点位于物体的表面，（除了数字误差的问题）从这个点出发的任何射线都会与物体本身相交。
        // 为什么这里需要做一点修正呢？为什么light_dir * N < 0就point - N * 1e-3？
        // 而且light_dir * N < 0这个条件不能反过来，要不然会出现很多黑点
        // 因为如果shadow_orig=point的话，那么会导致从shadow_orig出发就直接又碰到了point了，导致这个逻辑无法执行
        // 而且为什么要<0的时候要point - N*1e-3，主要是因为后面有段计算距离的逻辑，为了让light_dir*N < 0的时候一定是距离变短的，就故意减去一部分了
        Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // checking if the point lies in the shadow of the lights[i]
        Vec3f shadow_pt, shadow_N;
        Material tmpmaterial;
        // 从shadow_orig沿着light_dir出发，如果遇到其他物体，那么shadow_pt会有值，导致shadow_pt - shadow_orig的距离会比light_distance短
        if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_orig).norm() < light_distance)
            continue;

        // 光的入射角度和从圆心射出道交集点的向量的点乘
        diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
        // Ls = ks (I/r2) max(0, cos<>)^p, 这个版本里面没有前面的ks(I/r2)这块
        specular_light_intensity += powf(std::max(0.f, -reflect(-light_dir, N) * dir), material.specular_exponent) * lights[i].intensity;
    }
    // reflect_color * material.albedo[2] + refract_color * material.albedo[3];
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1];
}

void render(const std::vector<Sphere> &spheres, const std::vector<Light> &lights)
{
    const int width = 1024;
    const int height = 768;
    const int fov = M_PI / 2.;
    std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
    for (size_t j = 0; j < height; j++)
    {
        for (size_t i = 0; i < width; i++)
        {
            float x = (2 * (i + 0.5) / (float)width - 1) * tan(fov / 2.) * width / (float)height;
            float y = -(2 * (j + 0.5) / (float)height - 1) * tan(fov / 2.);
            // 这里必须要正则化
            Vec3f dir = Vec3f(x, y, -1).normalize();
            framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), dir, spheres, lights);
        }
    }

    std::ofstream ofs; // save the framebuffer to file
    ofs.open("./out.ppm");
    // ofs.open(filename, std::ofstream::out | std::ofstream::binary);
    ofs << "P6\n"
        << width << " " << height << "\n255\n";
    for (size_t i = 0; i < height * width; ++i)
    {
        Vec3f &c = framebuffer[i];
        float max = std::max(c[0], std::max(c[1], c[2]));
        // it prevents the colors from being brighter than maximum bright. 最大值不能超过1
        if (max > 1)
            c = c * (1. / max);
        for (size_t j = 0; j < 3; j++)
        {
            ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
        }
    }
    ofs.close();
}

int main()
{
    Material ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
    Material glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);
    Material red_rubber(1.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
    Material mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);

    std::vector<Sphere> spheres;
    spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
    spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
    spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
    spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));

    std::vector<Light> lights;
    lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
    lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
    lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

    render(spheres, lights);

    return 0;
}
