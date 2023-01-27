// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

/**
 * @brief cross product of v1v2 and v2v3
 * 
 * @param v1 
 * @param v2 
 * @param v3 
 * @return float 
 */
static float crossProduct(const Eigen::Vector2f v1, const Eigen::Vector2f v2, const Eigen::Vector2f v3)
{
    float v12_x = v2.x() - v1.x();
    float v12_y = v2.y() - v1.y();
    float v23_x = v3.x() - v2.x();
    float v23_y = v3.y() - v2.y();
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
static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // std::cout <<_v[0].x() << "\n";
    Eigen::Vector2f va = Eigen::Vector2f(_v[0].x(), _v[0].y());
    Eigen::Vector2f vb = Eigen::Vector2f(_v[1].x(), _v[1].y());
    Eigen::Vector2f vc = Eigen::Vector2f(_v[2].x(), _v[2].y());
    Eigen::Vector2f vd = Eigen::Vector2f(x, y);
    float cp1 = crossProduct(va, vb, vd);
    float cp2 = crossProduct(vb, vc, vd);
    float cp3 = crossProduct(vc, va, vd);
    // std::cout << cp1 << cp2 << cp3 << "\n";
    // 这里暂不考虑精度问题
    if( cp1 == 0.0 || cp2 == 0.0 || cp3 == 0.0 ){
        return true;
    }
    return (cp1 > 0.0 && cp2 > 0.0 && cp3 > 0.0) || (cp1 < 0.0 && cp2 < 0.0 && cp3 < 0.0);
}

/**
 * @brief Get the Color Rate object
 * x和y是左下方坐标，不是中心
 * @param x 
 * @param y 
 * @param _v 
 * @return float 
 */
static float getColorRate(float x, float y, const Vector3f* _v) {
    int hit = 0;
    if( insideTriangle(x + 0.25f, y + 0.25f, _v) ){
        hit ++;
    }
    if( insideTriangle(x + 0.75f, y + 0.25f, _v) ){
        hit ++;
    }    
    if( insideTriangle(x + 0.25f, y + 0.75f, _v) ){
        hit ++;
    }
    if( insideTriangle(x + 0.75f, y + 0.75f, _v) ){
        hit ++;
    } 
    // std:: cout << x << " " << y << " " << (1.0f * hit / 4) << "\n";
    return 1.0f * hit / 4;
}

static bool insideTriangleTest(Eigen::Vector2f va, Eigen::Vector2f vb, Eigen::Vector2f vc, Eigen::Vector2f vd)
{
    float cp1 = crossProduct(va, vb, vd);
    float cp2 = crossProduct(vb, vc, vd);
    float cp3 = crossProduct(vc, va, vd);
    // std::cout << cp1 << cp2 << cp3 << "\n";
    // 这里暂不考虑精度问题
    if( cp1 == 0.0 || cp2 == 0.0 || cp3 == 0.0 ){
        return true;
    }
    return (cp1 > 0.0 && cp2 > 0.0 && cp3 > 0.0) || (cp1 < 0.0 && cp2 < 0.0 && cp3 < 0.0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

static float get_z_interpolated(float x, float y, const Triangle& t)
{
    auto v = t.toVector4();
    // 计算对应的z，这里要做深度缓存判断                
    // depth_buf初始化的是设定了全部是infinity
    auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // 这里z_interpolated计算出来对应的是真实的z
    z_interpolated *= w_reciprocal;
    // 这里为了方便判断depth_buf，直接用负数处理
    z_interpolated = -z_interpolated;
    return z_interpolated;
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Eigen::Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            set_pixel(point,line_color);
        }
    }
}

void rst::rasterizer::draw_test()
{
    Eigen::Vector2f va = Eigen::Vector2f(1.0f, 1.0f);
    Eigen::Vector2f vb = Eigen::Vector2f(2.0f, 1.0f);
    Eigen::Vector2f vc = Eigen::Vector2f(1.0f, 2.0f);

    // outside
    Eigen::Vector2f vd = Eigen::Vector2f(2.0f, 2.0f);
    // inside
    Eigen::Vector2f ve = Eigen::Vector2f(1.5f, 1.5f);

    // crossProduct
    float cp1 = crossProduct(va, vb, vd);
    float cp2 = crossProduct(vb, vc, vd);
    float cp3 = crossProduct(vc, va, vd);
    bool inside1 = insideTriangleTest(va, vb, vc, vd);
    std::cout << "cp outside:" << cp1 << " " << cp2 << " " << cp3 << " " << inside1 << "\n";
    

    cp1 = crossProduct(va, vb, ve);
    cp2 = crossProduct(vb, vc, ve);
    cp3 = crossProduct(vc, va, ve);
    inside1 = insideTriangleTest(va, vb, vc, ve);
    std::cout << "cp inside:" << cp1 << " " << cp2 << " " << cp3 << " " <<  inside1 << "\n";
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;

    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            std::cout << "before vert.x:" << vert.x() << "\n";
            // std::cout << "before vert.y:" << vert.y() << "\n";
            // std::cout << "before vert.z:" << vert.z() << "\n";
            // x, y, z经过mvp处理之后是[-1, 1]的范围
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
            std::cout << "after vert.x:" << vert.x() << "\n";
            // std::cout << "after vert.y:" << vert.y() << "\n";
            // std::cout << "after vert.z:" << vert.z() << "\n"; 
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
        // 单线的渲染是对的
        // rasterize_wireframe(t);
    }
    rasterize_super_sample();
}

void rst::rasterizer::rasterize_super_sample()
{
    for(int x = 0; x < width; x++) {
        for(int y = 0; y < height; y++) {
            int index = get_index(x, y);
            int sample_index = index * 4;
            frame_buf[index] = (sample_frame_buf[sample_index] + sample_frame_buf[sample_index + 1] + sample_frame_buf[sample_index + 2] + sample_frame_buf[sample_index + 3]) / 4;
        }
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    draw_line(t.v[0], t.v[1]);
    draw_line(t.v[1], t.v[2]);
    draw_line(t.v[2], t.v[0]);
}

/**
 * 
 * @brief Screen space rasterization
 * super-sampling
 * 把一个像素当做2x2的格子，然后计算里面有多少个小格子在三角形里面，计算出比例作为颜色的深浅比例
 * @param t 
 */
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4(); 
    // std::cout << "v0:" << v.at(0).x() << "," << v.at(0).y() << "," << v.at(0).z() << std::endl;
    // std::cout << "v1:" << v.at(1).x() << "," << v.at(1).y() << "," << v.at(1).z() << std::endl;
    // std::cout << "v2:" << v.at(2).x() << "," << v.at(2).y() << "," << v.at(2).z() << std::endl;

    // TODO : Find out the bounding box of current triangle.    
    float x_arr[] = {v.at(0).x(), v.at(1).x(), v.at(2).x()};
    float y_arr[] = {v.at(0).y(), v.at(1).y(), v.at(2).y()};
    int min_x = std::floor(static_cast<double>(*std::min_element(std::begin(x_arr), std::end(x_arr))));
    int max_x = std::ceil(static_cast<double>(*std::max_element(std::begin(x_arr), std::end(x_arr))));
    int min_y = std::floor(static_cast<double>(*std::min_element(std::begin(y_arr), std::end(y_arr))));
    int max_y = std::ceil(static_cast<double>(*std::max_element(std::begin(y_arr), std::end(y_arr))));

    // std::cout << "min_x:" << min_x << std::endl;
    // std::cout << "max_x:" << max_x << std::endl;
    // std::cout << "min_y:" << min_y << std::endl;
    // std::cout << "max_y:" << max_y << std::endl;

    std::array<Eigen::Vector3f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector3f(vec.x(), vec.y(), vec.z()); });

    // iterate through the pixel and find if the current pixel is inside the triangle
    for( int x = min_x; x<=max_x; x++ ) {
        for( int y = min_y; y<=max_y; y++ ) {
            // std::cout << "x:" << x << " y:" << y << std::endl;
            // 这里x和y代表的是左下方的坐标，其实中心因为是在x+0.5和y+0.5
            // 黑边的理由，这里的判断is_inside的颗粒度是一个像素，但是这里会有很多in一个小格的点，但是一个像素是被判定为out的情况，所以会有黑边
            // bool is_inside = insideTriangle(1.0f * (x + 0.5f), 1.0f * (y + 0.5f), res.data());
            int index = get_index(x, y);
            Eigen::Vector3f color = t.getColor();
            int sample_index = 0;
            float z_interpolated = 0.0f, old_z;
            if( insideTriangle(x + 0.25f, y + 0.25f, res.data()) ){
                z_interpolated = get_z_interpolated(x + 0.25f, y + 0.25f, t);
                sample_index = index * 4;
                old_z = sample_depth_buf[sample_index];
                if( z_interpolated < old_z ){
                    sample_depth_buf[sample_index] = z_interpolated;
                    set_sample_pixel(sample_index, color);
                }
            }            
            if( insideTriangle(x + 0.75f, y + 0.25f, res.data()) ){
                z_interpolated = get_z_interpolated(x + 0.75f, y + 0.25f, t);
                sample_index = index * 4 + 1;
                old_z = sample_depth_buf[sample_index];
                if( z_interpolated < old_z ){
                    sample_depth_buf[sample_index] = z_interpolated;
                    set_sample_pixel(sample_index, color);
                }
            }    
            if( insideTriangle(x + 0.25f, y + 0.75f, res.data()) ){
                z_interpolated = get_z_interpolated(x + 0.25f, y + 0.75f, t);
                sample_index = index * 4 + 2;
                old_z = sample_depth_buf[sample_index];
                if( z_interpolated < old_z ){
                    sample_depth_buf[sample_index] = z_interpolated;
                    set_sample_pixel(sample_index, color);
                }
            }
            if( insideTriangle(x + 0.75f, y + 0.75f, res.data()) ){
                z_interpolated = get_z_interpolated(x + 0.75f, y + 0.75f, t);
                sample_index = index * 4 + 3;
                old_z = sample_depth_buf[sample_index];
                if( z_interpolated < old_z ){
                    sample_depth_buf[sample_index] = z_interpolated;
                    set_sample_pixel(sample_index, color);
                }
            }
            // std::cout << "is_inside:" << is_inside << " " << x << " " << y << std::endl;
            // if( is_inside ){                
            //     float z_interpolated = get_z_interpolated(1.0f * x, 1.0f * y, t);
            //     int index = get_index(x, y);             
            //     float old_z = depth_buf[index];
            //     // 当前z的比旧的小，需要进行更新
            //     if( z_interpolated < old_z ){
            //         depth_buf[index] = z_interpolated;
            //         Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
            //         // set_pixel(point, t.getColor());
            //         Eigen::Vector3f color = t.getColor();
            //         float rate = getColorRate(1.0f * x, 1.0f * y, res.data());
            //         Eigen::Vector3f newColor = Eigen::Vector3f(static_cast<int>(color.x()*rate), static_cast<int>(color.y()*rate), static_cast<int>(color.z()*rate));
            //         set_pixel(point, newColor);
            //     }
            // }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(sample_frame_buf.begin(), sample_frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    // 这里初始化的时候设定默认深度是infinity，也就是说越小的z离相机越近，但是相机是往-z的方向看的，所以需要特殊处理
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(sample_depth_buf.begin(), sample_depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    sample_frame_buf.resize(4 * w * h);
    depth_buf.resize(w * h);
    sample_depth_buf.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

/**
 * @brief 
 * 
 * @param point 传入的point的都是整数，代表的是屏幕中的某一个点，这里像素的中心好像不是[+0.5]，而是刚好是[0]
 * @param color 
 */
void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    // std::cout << "ind:" << point.y() << " " << point.x() << " " << ind << std::endl;
    frame_buf[ind] = color;

}

void rst::rasterizer::set_sample_pixel(int sample_index, const Eigen::Vector3f& color)
{
    sample_frame_buf[sample_index] = color;
}

// clang-format on