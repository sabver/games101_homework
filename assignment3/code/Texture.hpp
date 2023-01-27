//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColor2(float u, float v)
    {
        auto color = image_data.at<cv::Vec3b>(v, u);
        
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        // std::cout << "width: " << width << "height: " << height << " u:" << u_img << " v:" << v_img << std::endl;
        Eigen::Vector2f u00 = Eigen::Vector2f(std::floor(u_img), std::floor(v_img));
        Eigen::Vector2f u01 = Eigen::Vector2f(std::floor(u_img), std::ceil(v_img));
        Eigen::Vector2f u10 = Eigen::Vector2f(std::ceil(u_img), std::floor(v_img));
        Eigen::Vector2f u11 = Eigen::Vector2f(std::ceil(u_img), std::ceil(v_img));
        // std::cout << "1" << std::endl;
        float s = u_img - std::floor(u_img);
        float t = v_img - std::floor(v_img);
        // std::cout << "2" << std::endl;
        // u00, u10
        Eigen::Vector3f u0 = lerp(s, getColor2(u00.x(), u00.y()), getColor2(u10.x(), u10.y()));
        // u01, u11
        Eigen::Vector3f u1 = lerp(s, getColor2(u01.x(), u01.y()), getColor2(u11.x(), u11.y()));
        // std::cout << "3" << std::endl;
        Eigen::Vector3f color = lerp(t, u0, u1);
        // std::cout << "4" << std::endl;
        return color;
    }

    Eigen::Vector3f lerp(const float& t, const Eigen::Vector3f& a, const Eigen::Vector3f& b)
    {
        // std::cout << "lerp: " << t << " " << a << " " << b << std::endl;
        return a * (1 - t) + b * t;
    }    
};
#endif // RASTERIZER_TEXTURE_H
