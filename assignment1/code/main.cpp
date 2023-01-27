#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <cmath>

constexpr double MY_PI = 3.1415926;

// Find a good angle to put the camera(view transformation)
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    // eye_pos是相机的位置
    // translate只是把相机变换回原点
    // 这里还没有考虑到rotation的处理
    Eigen::Matrix4f translate;
    // 这里的写法是横过来的
    // 1, 0, 0, -eye_pos[0]
    // 0, 1, 0, -eye_pos[1]
    // 0, 0, 1, -eye_pos[2]
    // 0, 0, 0, 1
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

// Find a good place and arrange people(model transformation)
// 这次作业只是先让三角形旋转而已，算是一个预处理吧
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    // std::cout << "get model matrix rotation angle:" << rotation_angle << "\n";
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    const double PI = acos(-1);
    const float cos_val = cos(rotation_angle * PI / 180);
    const float sin_val = sin(rotation_angle * PI / 180);
    // std::cout << "cos:" << cos_val << "\n";
    // std::cout << "sin:" << sin_val << "\n";
    Eigen::Matrix4f rotation;
    rotation << cos_val, -sin_val, 0, 0,
        sin_val, cos_val, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;
    // std::cout << "rotation:" << rotation << "\n";
    model = model * rotation;
    return model;
}

/**
 * @brief Get the projection matrix object
 *
 * The vertical field of view: the vertical angle of the camera through which we are looking at the world.
 * @param eye_fov 45 field-of-view
 * The aspect ratio - the ratio between the width and the height of the rectangular area which will be the target of projection.
 * @param aspect_ratio 1 aspect_ratio = width / height
 * @param zNear 0.1
 * @param zFar  50
 * @return Eigen::Matrix4f
 */
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    std::cout << "get_projection_matrix \n";
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
    std::cout << "persp_ortho:" << persp_ortho << "\n";
    // 这里默认了相机位置是在原点的，所以ortho_translate是不起作用的
    ortho_translate << 1, 0, 0, -abs(r + l) / 2,
        0, 1, 0, -abs(t + b) / 2,
        0, 0, 1, -abs(n + f) / 2,
        0, 0, 0, 1;
    std::cout << "ortho_translate:" << ortho_translate << "\n";
    ortho_scale << 2 / abs(r - l), 0, 0, 0,
        0, 2 / abs(t - b), 0, 0,
        0, 0, 2 / abs(n - f), 0,
        0, 0, 0, 1;
    std::cout << "ortho_scale:" << ortho_scale << "\n";
    projection = projection * ortho_translate * ortho_scale * persp_ortho;
    return projection;
}

/**
 * @brief
 * 这里的实现没有考虑到相机的z位置对成像的影响
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, const char **argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;
    float eye_fov = 45, aspect_ratio = 1, zNear = 0.1, zFar = 50;
    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(eye_fov, aspect_ratio, zNear, zFar));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 10;
        }
        else if (key == 'd')
        {
            angle -= 10;
        }
    }

    return 0;
}
