#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "geometry.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const int width = 200;
const int height = 200;

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

void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color)
{
    // std::cout << t0 << t1 << t2 << std::endl;
    // line(t0, t1, image, color);
    // line(t1, t2, image, color);
    // line(t2, t0, image, color);
    // Find out the bounding box of current triangle.
    float x_arr[] = {t0.x, t1.x, t2.x};
    float y_arr[] = {t0.y, t1.y, t2.y};
    int min_x = std::floor(static_cast<double>(*std::min_element(std::begin(x_arr), std::end(x_arr))));
    int max_x = std::ceil(static_cast<double>(*std::max_element(std::begin(x_arr), std::end(x_arr))));
    int min_y = std::floor(static_cast<double>(*std::min_element(std::begin(y_arr), std::end(y_arr))));
    int max_y = std::ceil(static_cast<double>(*std::max_element(std::begin(y_arr), std::end(y_arr))));
    for (int x = min_x; x <= max_x; x++)
    {
        for (int y = min_y; y <= max_y; y++)
        {
            bool is_inside = insideTriangle(x, y, t0, t1, t2);
            if (is_inside)
            {
                image.set(x, y, color);
            }
        }
    }
}

int main(int argc, char **argv)
{
    TGAImage image(width, height, TGAImage::RGB);

    Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
    Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
    Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};

    triangle(t0[0], t0[1], t0[2], image, red);
    triangle(t1[0], t1[1], t1[2], image, white);
    triangle(t2[0], t2[1], t2[2], image, green);

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}
