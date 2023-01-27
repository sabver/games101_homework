#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include <math.h> 

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    Eigen::Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Eigen::Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    // TODO: Use the same projection matrix from the previous assignments
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    // 因为相机是往-z的方向看，所以这里设定为负数
    // 如果这里符号没处理好的话，会导致渲染出来的三角形会上下颠倒
    float n = -abs(zNear), f = -abs(zFar);
    // 这里需要控制好正负
    float t = tan(eye_fov / 2) * abs(n), b = -t;
    float r = t * aspect_ratio, l = -r;

    Eigen::Matrix4f persp_ortho, ortho_translate, ortho_scale;
    persp_ortho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -(n * f),
        0, 0, 1, 0;
    // 这里默认了相机位置是在原点的，所以ortho_translate是不起作用的
    ortho_translate << 1, 0, 0, -abs(r + l) / 2,
        0, 1, 0, -abs(t + b) / 2,
        0, 0, 1, -abs(n + f) / 2,
        0, 0, 0, 1;
    ortho_scale << 2 / abs(r - l), 0, 0, 0,
        0, 2 / abs(t - b), 0, 0,
        0, 0, 2 / abs(n - f), 0,
        0, 0, 0, 1;
    projection = projection * ortho_translate * ortho_scale * persp_ortho;
    return projection;
}

Eigen::Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Eigen::Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}

struct light
{
    Eigen::Vector3f position;
    Eigen::Vector3f intensity;
};

Eigen::Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
        // std::cout << payload.tex_coords << " " << payload.tex_coords[0] << " " << payload.tex_coords[1] << " " << std::endl;
        // return_color = payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1]);
        // 测试双线性插值进行纹理采样
        return_color = payload.texture->getColorBilinear(payload.tex_coords[0], payload.tex_coords[1]);
        // std::cout << return_color << std::endl;
    }
    Eigen::Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = texture_color / 255.f;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = texture_color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    // 观察向量
    Eigen::Vector3f v = (eye_pos - point).normalized();
    // 环境变量
    Eigen::Vector3f la = ka.cwiseProduct(amb_light_intensity);
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f light_vec = (light.position - point).normalized();
        float r = (light.position - point).norm();
        Eigen::Vector3f light_power = (light.intensity / std::pow(r, 2.0f));
        float dot = light_vec.dot(normal);
        dot = dot > 0 ? dot : 0;
        // 漫反射
        Eigen::Vector3f ld = kd.cwiseProduct(light_power) * std::max(0.0f, light_vec.dot(normal));
        // 镜面反射
        // 半程向量
        Eigen::Vector3f h = (light_vec + v).normalized();
        Eigen::Vector3f ls = ks.cwiseProduct(light_power) * std::pow(std::max(0.0f, h.dot(normal)), p);

        result_color += (la + ld + ls);
    }

    return result_color * 255.f;
}

/**
 * @brief 
 * 光源位置，着色顶点（normal），相机位置
 * payload.view_pos就是顶点经过mv变换之后的结果（相机坐标的数值），payload.normal也是相机坐标的数值
 * 而一开始给出的光源位置和相机位置都是相机坐标的？这里不太明白？就先当做是相机坐标的
 * @param payload 
 * @return Eigen::Vector3f 
 */
Eigen::Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005); // 环境反射系数
    Eigen::Vector3f kd = payload.color; // 漫反射系数
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937); // 镜面反射系数

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color;
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    Eigen::Vector3f result_color = {0, 0, 0};
    // 观察向量
    Eigen::Vector3f v = (eye_pos - point).normalized();
    // 环境变量
    Eigen::Vector3f la = ka.cwiseProduct(amb_light_intensity);
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f light_vec = (light.position - point).normalized();
        float r = (light.position - point).norm();
        Eigen::Vector3f light_power = (light.intensity / std::pow(r, 2.0f));
        // 漫反射
        Eigen::Vector3f ld = kd.cwiseProduct(light_power) * std::max(0.0f, light_vec.dot(normal));
        // 镜面反射
        // 半程向量
        Eigen::Vector3f h = (light_vec + v).normalized();
        Eigen::Vector3f ls = ks.cwiseProduct(light_power) * std::pow(std::max(0.0f, h.dot(normal)), p);

        result_color += (la + ld + ls);
    }

    return result_color * 255.f;
}



Eigen::Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal;
    float x = normal.x(), y = normal.y(), z = normal.z();
    // sqrt(x*x+z*z)
    float s = sqrt(x * x + z * z);
    float tx = x * y / s, ty = s, tz = z * y / s;
    Eigen::Vector3f t{tx, ty, tz};
    Eigen::Vector3f b = normal.cross(t);
    // t b n 分别是竖着从左到右放的
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), n.x(),
            t.y(), b.y(), n.y(),
            t.z(), b.z(), n.z();
    // payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1])
    // bump mapping 部分的 h(u,v)=texture_color(u,v).norm, 其中 u,v 是 tex_coords, w,h 是 texture 的宽度与高度
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;
    float huv = payload.texture->getColor(u, v).norm();
    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - huv);
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - huv);

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    point = point + kn * n * huv;
    normal = (TBN * ln).normalized();
    
    Eigen::Vector3f result_color = {0, 0, 0};
    // 观察向量
    Eigen::Vector3f v_vec = (eye_pos - point).normalized();
    // 环境变量
    Eigen::Vector3f la = ka.cwiseProduct(amb_light_intensity);
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Eigen::Vector3f light_vec = (light.position - point).normalized();
        float r = (light.position - point).norm();
        Eigen::Vector3f light_power = (light.intensity / std::pow(r, 2.0f));
        // 漫反射
        Eigen::Vector3f ld = kd.cwiseProduct(light_power) * std::max(0.0f, light_vec.dot(normal));
        // 镜面反射
        // 半程向量
        Eigen::Vector3f h = (light_vec + v_vec).normalized();
        Eigen::Vector3f ls = ks.cwiseProduct(light_power) * std::pow(std::max(0.0f, h.dot(normal)), p);

        result_color += (la + ld + ls);
    }

    return result_color * 255.f;
}


Eigen::Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
    Eigen::Vector3f kd = payload.color;
    Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    std::vector<light> lights = {l1, l2};
    Eigen::Vector3f amb_light_intensity{10, 10, 10};
    Eigen::Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Eigen::Vector3f color = payload.color; 
    Eigen::Vector3f point = payload.view_pos;
    Eigen::Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    Eigen::Vector3f n = normal;
    float x = normal.x(), y = normal.y(), z = normal.z();
    // sqrt(x*x+z*z)
    float s = sqrt(x * x + z * z);
    float tx = x * y / s, ty = s, tz = z * y / s;
    Eigen::Vector3f t{tx, ty, tz};
    Eigen::Vector3f b = normal.cross(t);
    // t b n 分别是竖着从左到右放的
    Eigen::Matrix3f TBN;
    TBN << t.x(), b.x(), n.x(),
            t.y(), b.y(), n.y(),
            t.z(), b.z(), n.z();
    // payload.texture->getColor(payload.tex_coords[0], payload.tex_coords[1])
    // bump mapping 部分的 h(u,v)=texture_color(u,v).norm, 其中 u,v 是 tex_coords, w,h 是 texture 的宽度与高度
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();
    float w = payload.texture->width;
    float h = payload.texture->height;

    float dU = kh * kn * (payload.texture->getColor(u + 1.0f / w, v).norm() - payload.texture->getColor(u, v).norm());
    float dV = kh * kn * (payload.texture->getColor(u, v + 1.0f / h).norm() - payload.texture->getColor(u, v).norm());

    Eigen::Vector3f ln = Eigen::Vector3f(-dU, -dV, 1.0f);
    normal = (TBN * ln).normalized();
    Eigen::Vector3f result_color = {0, 0, 0};
    result_color = normal;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                // 纹理是一开始做模型的时候就定好的？是的
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            // std::cout << "vertex:" << t->v[0] << " " << t->v[1] << " " << t->v[2] << " " << std::endl;
            // std::cout << "normal:" << t->normal[0] << " " << t->normal[1] << " " << t->normal[2] << std::endl;
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    std::function<Eigen::Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = std::string(argv[1]);

        if (argc == 3 && std::string(argv[2]) == "texture")
        {
            std::cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && std::string(argv[2]) == "normal")
        {
            std::cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "phong")
        {
            std::cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "bump")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && std::string(argv[2]) == "displacement")
        {
            std::cout << "Rasterizing using the bump shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Eigen::Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
