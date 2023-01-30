#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const int width = 800;
const int height = 800;
int ybuffer[width];
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

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P)
{
    Vec3f s[2];
    for (int i = 2; i--;)
    {
        s[i][0] = C[i] - A[i];
        s[i][1] = B[i] - A[i];
        s[i][2] = A[i] - P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2]) > 1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f - (u.x + u.y) / u.z, u.y / u.z, u.x / u.z);
    return Vec3f(-1, 1, 1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}

/**
 * @brief
 * @param pts
 * @param texture_ps 三个顶点对应的纹理坐标
 * @param image
 * @param color
 */
void triangle(Vec3f *pts, Vec3f *texture_ps, float *zbuffer, TGAImage &image, TGAImage texture, float intensity)
{
    // std::cout << "triangle" << std::endl;
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; i++)
    {
        bboxmin.x = std::max(0.f, std::min(bboxmin.x, pts[i].x));
        bboxmin.y = std::max(0.f, std::min(bboxmin.y, pts[i].y));

        bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
        bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
    }
    Vec3f P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++)
    {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++)
        {
            // bc_screen就是重心坐标
            // 这里的根据P.x和P.y来求三维空间对应的重心坐标需要梳理一下
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
                continue;
            // 对z进行插值
            P.z = pts[0].z * bc_screen.x + pts[1].z * bc_screen.y + pts[2].z * bc_screen.z;
            int index = int(P.x + P.y * width);
            if (zbuffer[index] < P.z)
            {
                zbuffer[index] = P.z;
                // 对纹理的x进行插值
                float texture_x = texture_ps[0].x * bc_screen.x + texture_ps[1].x * bc_screen.y + texture_ps[2].x * bc_screen.z;
                // 对纹理的y进行插值
                float texture_y = texture_ps[0].y * bc_screen.x + texture_ps[1].y * bc_screen.y + texture_ps[2].y * bc_screen.z;
                // 计算出对应具体纹理里面的x和y
                int x = (int)(texture_x * texture.get_width()), y = (int)(texture_y * texture.get_height());
                // std::cout << texture_x << " " << texture_y << " " << x << " " << y << std::endl;
                // 得到纹理的颜色
                TGAColor color = texture.get(x, y);
                image.set(P.x, P.y, TGAColor(intensity * color.r, intensity * color.g, intensity * color.b, 255));
            }
        }
    }
}

void triangle_simple(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color)
{
    Vec2f bboxmin(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width() - 1, image.get_height() - 1);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
            bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));
        }
    }
    Vec3f P;
    for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++)
    {
        for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++)
        {
            Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
            if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
                continue;
            P.z = 0;
            for (int i = 0; i < 3; i++)
                P.z += pts[i][2] * bc_screen[i];
            if (zbuffer[int(P.x + P.y * width)] < P.z)
            {
                zbuffer[int(P.x + P.y * width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

/**
 * @brief 把二维的图像都投影到x轴上，需要做y的buffer
 *
 * @param p0
 * @param p1
 * @param image
 * @param color
 * @param ybuffer
 */
void rasterize(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color, int ybuffer[])
{
    if (p0.x > p1.x)
    {
        std::swap(p0, p1);
    }
    for (int x = p0.x; x <= p1.x; x++)
    {
        float t = (x - p0.x) / (float)(p1.x - p0.x);
        // 线性插值计算出对应的y
        int y = p0.y * (1. - t) + p1.y * t;
        if (ybuffer[x] < y)
        {
            ybuffer[x] = y;
            image.set(x, 0, color);
        }
    }
}

/**
 * @brief
 * 这里的做法不太明白
 * @param v
 * @return Vec3f
 */
Vec3f world2screen(Vec3f v)
{
    return Vec3f(int((v.x + 1.) * width / 2. + .5), int((v.y + 1.) * height / 2. + .5), v.z);
}

int main(int argc, char **argv)
{

    if (2 == argc)
    {
        model = new Model(argv[1]);
    }
    else
    {
        model = new Model("obj/african_head.obj");
    }
    // std::cout << "end model" << std::endl;

    // for (int i = 0; i < width; i++)
    // {
    //     ybuffer[i] = std::numeric_limits<int>::min();
    // }

    // line(Vec2i(20, 34), Vec2i(744, 400), image, red);
    // line(Vec2i(120, 434), Vec2i(444, 400), image, green);
    // line(Vec2i(330, 463), Vec2i(594, 200), image, blue);
    // rasterize(Vec2i(20, 34), Vec2i(744, 400), image, red, ybuffer);
    // rasterize(Vec2i(120, 434), Vec2i(444, 400), image, green, ybuffer);
    // rasterize(Vec2i(330, 463), Vec2i(594, 200), image, blue, ybuffer);

    float *zbuffer = new float[width * height];
    for (int i = width * height; i--; zbuffer[i] = -std::numeric_limits<float>::max())
        ;

    // 读取纹理文件，这里的width，height等参数没意义
    TGAImage texture(width, height, TGAImage::RGB);
    texture.read_tga_file("obj/african_head_diffuse.tga");
    texture.flip_vertically();

    TGAImage image(width, height, TGAImage::RGB);
    Vec3f light_dir(0, 0, -1);
    for (int i = 0; i < model->nfaces(); i++)
    {
        std::vector<int> face = model->face(i), face_texture = model->face_texture(i);

        Vec3f pts[3], texture_ps[3], world_coords[3];
        for (int j = 0; j < 3; j++)
        {
            pts[j] = world2screen(model->vert(face[j]));
            texture_ps[j] = model->vert_texture(face_texture[j]);
            world_coords[j] = model->vert(face[j]);
        }
        Vec3f n = cross((world_coords[2] - world_coords[0]), (world_coords[1] - world_coords[0]));
        n.normalize();
        float intensity = n * light_dir;
        // 这里输出的图像里面不像是多个三角形组成的，会不会是因为没有加入光照这个要素？
        triangle(pts, texture_ps, zbuffer, image, texture, intensity);
        // triangle_simple(pts, zbuffer, image, TGAColor(rand() % 255, rand() % 255, rand() % 255, 255));
    }

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    return 0;
}
