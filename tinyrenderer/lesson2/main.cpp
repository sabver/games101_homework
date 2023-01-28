#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 200;
const int height = 200;
Model *model = NULL;

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color)
{
    bool steep = false;
    if (std::abs(p0.x - p1.x) < std::abs(p0.y - p1.y))
    {
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x > p1.x)
    {
        std::swap(p0, p1);
    }

    for (int x = p0.x; x <= p1.x; x++)
    {
        float t = (x - p0.x) / (float)(p1.x - p0.x);
        int y = p0.y * (1. - t) + p1.y * t;
        if (steep)
        {
            image.set(y, x, color);
        }
        else
        {
            image.set(x, y, color);
        }
    }
}

/**
 * @brief cross product of v1v2 and v2v3
 *
 * @param v1
 * @param v2
 * @param v3
 * @return float
 */
static float crossProduct(const Vec2i v1, const Vec2i v2, const Vec2i v3)
{
    float v12_x = v2.x - v1.x;
    float v12_y = v2.y - v1.y;
    float v23_x = v3.x - v2.x;
    float v23_y = v3.y - v2.y;
    return v12_x * v23_y - v23_x * v12_y;
}

/**
 * @brief
 * 如果cross product等于0，也就是(x, y)刚好在三角形边上，这种情况判断为true
 *
 * @param x
 * @param y
 * @param _v
 * @return true
 * @return false
 */
static bool insideTriangle(float x, float y, Vec2i t0, Vec2i t1, Vec2i t2)
{
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // std::cout <<_v[0].x() << "\n";
    Vec2i p = Vec2i(x, y);
    float cp1 = crossProduct(t0, t1, p);
    float cp2 = crossProduct(t1, t2, p);
    float cp3 = crossProduct(t2, t0, p);
    // std::cout << cp1 << cp2 << cp3 << "\n";
    // 这里暂不考虑精度问题
    if (cp1 == 0.0 || cp2 == 0.0 || cp3 == 0.0)
    {
        return true;
    }
    return (cp1 > 0.0 && cp2 > 0.0 && cp3 > 0.0) || (cp1 < 0.0 && cp2 < 0.0 && cp3 < 0.0);
}

// void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
// {
//     // std::cout << t0 << t1 << t2 << std::endl;
//     // line(t0, t1, image, color);
//     // line(t1, t2, image, color);
//     // line(t2, t0, image, color);
//     // Find out the bounding box of current triangle.
//     float x_arr[] = {t0.x, t1.x, t2.x};
//     float y_arr[] = {t0.y, t1.y, t2.y};
//     int min_x = std::floor(static_cast<double>(*std::min_element(std::begin(x_arr), std::end(x_arr))));
//     int max_x = std::ceil(static_cast<double>(*std::max_element(std::begin(x_arr), std::end(x_arr))));
//     int min_y = std::floor(static_cast<double>(*std::min_element(std::begin(y_arr), std::end(y_arr))));
//     int max_y = std::ceil(static_cast<double>(*std::max_element(std::begin(y_arr), std::end(y_arr))));
//     for (int x = min_x; x <= max_x; x++)
//     {
//         for (int y = min_y; y <= max_y; y++)
//         {
//             bool is_inside = insideTriangle(x, y, t0, t1, t2);
//             if (is_inside)
//             {
//                 image.set(x, y, color);
//             }
//         }
//     }
// }

// 这里的计算公式需要额外找wiki看
Vec3f barycentric(Vec2i *pts, Vec2i P)
{
    Vec3f u = Vec3f(pts[2].x - pts[0].x, pts[1].x - pts[0].x, pts[0].x - P.x) ^ Vec3f(pts[2].y - pts[0].y, pts[1].y - pts[0].y, pts[0].y - P.y);
    /* `pts` and `P` has integer value as coordinates
       so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z) < 1)
        return Vec3f(-1, 1, 1);
    return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
}

/**
 * @brief 这种画三角形的方式是比较容易理解的方式，但是很难并行处理
 * 为了可以并行处理，有另外一种方法，就是给三角形上下分开，然后上下两个三角形各自从左到右画直线即可，可以两个线程分开画上下两部分
 * @param pts
 * @param image
 * @param color
 */
void triangle(Vec2i *pts, TGAImage &image, TGAColor color)
{
    Vec2i bboxmin(image.get_width() - 1, image.get_height() - 1);
    Vec2i bboxmax(0, 0);
    Vec2i clamp(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; i++)
    {
        bboxmin.x = std::max(0, std::min(bboxmin.x, pts[i].x));
        bboxmin.y = std::max(0, std::min(bboxmin.y, pts[i].y));

        bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
        bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
    }
    Vec2i P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++)
    {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++)
        {
            Vec3f bc_screen = barycentric(pts, P);
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
                continue;
            image.set(P.x, P.y, color);
        }
    }
}

// 可以并行的版本
// https://github.com/ssloy/tinyrenderer/wiki/Lesson-2:-Triangle-rasterization-and-back-face-culling
// void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color) {
//     // sort the vertices, t0, t1, t2 lower−to−upper (bubblesort yay!)
//     if (t0.y>t1.y) std::swap(t0, t1);
//     if (t0.y>t2.y) std::swap(t0, t2);
//     if (t1.y>t2.y) std::swap(t1, t2);
//     int total_height = t2.y-t0.y;
//     for (int y=t0.y; y<=t1.y; y++) {
//         int segment_height = t1.y-t0.y+1;
//         float alpha = (float)(y-t0.y)/total_height;
//         float beta  = (float)(y-t0.y)/segment_height; // be careful with divisions by zero
//         Vec2i A = t0 + (t2-t0)*alpha;
//         Vec2i B = t0 + (t1-t0)*beta;
//         if (A.x>B.x) std::swap(A, B);
//         for (int j=A.x; j<=B.x; j++) {
//             image.set(j, y, color); // attention, due to int casts t0.y+i != A.y
//         }
//     }
//     for (int y=t1.y; y<=t2.y; y++) {
//         int segment_height =  t2.y-t1.y+1;
//         float alpha = (float)(y-t0.y)/total_height;
//         float beta  = (float)(y-t1.y)/segment_height; // be careful with divisions by zero
//         Vec2i A = t0 + (t2-t0)*alpha;
//         Vec2i B = t1 + (t2-t1)*beta;
//         if (A.x>B.x) std::swap(A, B);
//         for (int j=A.x; j<=B.x; j++) {
//             image.set(j, y, color); // attention, due to int casts t0.y+i != A.y
//         }
//     }
// }

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);

    // Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
    // Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
    // Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};

    // triangle(t0[0], t0[1], t0[2], image, red);
    // triangle(t1[0], t1[1], t1[2], image, white);
    // triangle(t2[0], t2[1], t2[2], image, green);

    // TGAImage frame(200, 200, TGAImage::RGB);
    // Vec2i pts[3] = {Vec2i(10, 10), Vec2i(100, 30), Vec2i(190, 160)};
    // triangle(pts, frame, TGAColor(255, 0, 0, 0));

    if (2 == argc)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head.obj");
    }

    // 这里为了给三角形着色，利用一个原理，和光源越垂直的越亮
    Vec3f light_dir(0, 0, -1); // define light_dir

    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];
        Vec3f world_coords[3];
        for (int j = 0; j < 3; j++)
        {
            Vec3f v = model->vert(face[j]);
            screen_coords[j] = Vec2i((v.x + 1.) * width / 2., (v.y + 1.) * height / 2.);
            world_coords[j] = v;
        }
        Vec3f n = (world_coords[2] - world_coords[0]) ^ (world_coords[1] - world_coords[0]);
        n.normalize();
        float intensity = n * light_dir;
        if (intensity)
        {
            triangle(screen_coords, image, TGAColor(intensity * 255, intensity * 255, intensity * 255, 255));
        }
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}
