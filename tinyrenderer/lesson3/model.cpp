#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_(), vert_textures_(), face_textures_()
{
    // std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << "# vt#" << vert_textures_.size() << " ft# " << face_textures_.size() << std::endl;
    // std::cout << "test" << std::endl;
    std::ifstream in;
    in.open(filename, std::ifstream::in);
    if (in.fail())
        return;
    std::string line;
    while (!in.eof())
    {
        std::getline(in, line);
        // std::cout << line.c_str() << std::endl;
        std::istringstream iss(line.c_str());
        // std::cout << line.c_str() << std::endl;
        char trash;
        if (!line.compare(0, 2, "v "))
        {
            iss >> trash;
            Vec3f v;
            for (int i = 0; i < 3; i++)
                iss >> v[i];
            verts_.push_back(v);
        }
        else if (!line.compare(0, 2, "f "))
        {
            // 用f_texture来接收纹理的坐标下标
            std::vector<int> f, f_texture;
            int itrash, idx, idx_texture;
            iss >> trash;
            while (iss >> idx >> trash >> idx_texture >> trash >> itrash)
            {
                idx--; // in wavefront obj all indices start at 1, not zero
                idx_texture--;
                f.push_back(idx);
                f_texture.push_back(idx_texture);
            }
            faces_.push_back(f);
            face_textures_.push_back(f_texture);
            // std::cout << f[0] << "," << f[1] << "," << f[2] << std::endl;
        }
        else if (!line.compare(0, 3, "vt ")) // vt的构造和v的构造一样
        {
            // std::cout << "xxx" << std::endl;
            iss >> trash;
            iss >> trash;
            // std::cout << trash << std::endl;
            Vec3f v;
            // 这里无法处理0.000的情况然后导致报错，后面的0.000直接不处理，因为是二维的
            for (int i = 0; i < 2; i++)
            {                
                iss >> v[i];
                // std::cout << v[i] << std::endl;              
            }
            // std::cout << "end" << std::endl;
            v[2] = 0.f;
            // std::cout << "end2" << v << std::endl;
            vert_textures_.push_back(v);
            // std::cout << v[0] << "," << v[1] << std::endl;
        }
    }
    std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << "# vt#" << vert_textures_.size() << " ft# " << face_textures_.size() << std::endl;
}

Model::~Model()
{
}

int Model::nverts()
{
    return (int)verts_.size();
}

int Model::nfaces()
{
    return (int)faces_.size();
}

int Model::nvert_textures()
{
    return (int)vert_textures_.size();
}

int Model::nface_textures()
{
    return (int)face_textures_.size();
}

std::vector<int> Model::face(int idx)
{
    return faces_[idx];
}

Vec3f Model::vert(int i)
{
    return verts_[i];
}

std::vector<int> Model::face_texture(int idx)
{
    return face_textures_[idx];
}

Vec3f Model::vert_texture(int i)
{
    return vert_textures_[i];
}