#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model *model = NULL;
const int width = 800;
const int height = 800;

Vec3f light_dir(1, 1, 1);
Vec3f eye(0, -1, 3);
Vec3f center(0, 0, 0);
Vec3f up(0, 1, 0);

struct GouraudShader : public IShader
{
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader

    /**
     * @brief
     *
     * @param iface
     * @param nthvert
     * @return Vec4f
     */
    virtual Vec4f vertex(int iface, int nthvert)
    {
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));                               // read the vertex from .obj file
        gl_Vertex = Viewport * Projection * ModelView * gl_Vertex;                             // transform it to screen coordinates
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert) * light_dir); // get diffuse lighting intensity
        return gl_Vertex;
    }
    /**
     * @brief 根据重心坐标计算出当前的颜色
     *
     * @param bar 重心坐标得出的三个插值参数
     * @param color 需要计算的颜色
     * @return true
     * @return false
     */
    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        float intensity = varying_intensity * bar;   // interpolate intensity for the current pixel
        color = TGAColor(255, 255, 255) * intensity; // well duh
        return false;                                // no, we do not discard this pixel
    }

    // Simple modification of the Gourad shading, where the intensities are allowed to have 6 values only
    // virtual bool fragment(Vec3f bar, TGAColor &color)
    // {
    //     float intensity = varying_intensity * bar;
    //     if (intensity > .85)
    //         intensity = 1;
    //     else if (intensity > .60)
    //         intensity = .80;
    //     else if (intensity > .45)
    //         intensity = .60;
    //     else if (intensity > .30)
    //         intensity = .45;
    //     else if (intensity > .15)
    //         intensity = .30;
    //     else
    //         intensity = 0;
    //     color = TGAColor(255, 155, 0) * intensity;
    //     return false;
    // }
};

/**
 * @brief Textures Shader
 *
 */
struct TextureShader : public IShader
{
    Vec3f varying_intensity;     // written by vertex shader, read by fragment shader
    mat<2, 3, float> varying_uv; // same as above

    virtual Vec4f vertex(int iface, int nthvert)
    {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert) * light_dir); // get diffuse lighting intensity
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert));                               // read the vertex from .obj file
        return Viewport * Projection * ModelView * gl_Vertex;                                  // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color)
    {
        float intensity = varying_intensity * bar; // interpolate intensity for the current pixel
        Vec2f uv = varying_uv * bar;               // interpolate uv for the current pixel
        color = model->diffuse(uv) * intensity;    // well duh
        return false;                              // no, we do not discard this pixel
    }
};
/**
 * @brief Specular mapping
 * 
 */
struct SpecularShader : public IShader {
    mat<2,3,float> varying_uv;  // same as above
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        return Viewport*Projection*ModelView*gl_Vertex; // transform it to screen coordinates
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec2f uv = varying_uv*bar;
        Vec3f n = proj<3>(uniform_MIT*embed<4>(model->normal(uv))).normalize();
        Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir        )).normalize();
        Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
        float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
        float diff = std::max(0.f, n*l);
        TGAColor c = model->diffuse(uv);
        color = c;
        for (int i=0; i<3; i++) color[i] = std::min<float>(5 + c[i]*(diff + .6*spec), 255);
        return false;
    }
};

/**
 * @brief
 * primitive processing routine
 * @param argc
 * @param argv
 * @return int
 */
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

    lookat(eye, center, up);
    viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
    projection(-1.f / (eye - center).norm());
    light_dir.normalize();

    TGAImage image(width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    // GouraudShader shader;
    // TextureShader shader;
    SpecularShader shader;
    shader.uniform_M   =  Projection*ModelView;
    shader.uniform_MIT = (Projection*ModelView).invert_transpose();
    for (int i = 0; i < model->nfaces(); i++)
    {
        Vec4f screen_coords[3];
        for (int j = 0; j < 3; j++)
        {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }

    image.flip_vertically(); // to place the origin in the bottom left corner of the image
    zbuffer.flip_vertically();
    image.write_tga_file("output.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}
