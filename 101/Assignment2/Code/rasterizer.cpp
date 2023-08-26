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

static float HeronArea(Vector2f& A, Vector2f& B, Vector2f& C)
{
    float side1 = sqrt(pow(A.x() - B.x(), 2) + pow(A.y()-B.y(), 2));
    float side2 = sqrt(pow(B.x() - C.x(), 2) + pow(B.y()-C.y(), 2));
    float side3 = sqrt(pow(C.x() - A.x(), 2) + pow(C.y()-A.y(), 2));

    float p = (side1 + side2 + side3) / 2.0;
    float area = sqrt(p*(p-side1)*(p-side2)*(p-side3));
    return area;
}
static float zAxisValue(Vector2f& P, Vector2f& vo, Vector2f& vd)
{
    Vector2f side, vec;
    side << vd.x() - vo.x(), vd.y() - vo.y();
    vec << P.x() - vo.x(), P.y() - vo.y();

    float z = side.x()*vec.y() - side.y()*vec.x();
    return z;
}
static void crossProduct(Vector3f& P, Vector3f& vo, Vector3f& vd, Vector3f& dir)
{
    Vector3f side, vec;
    side << vd.x() - vo.x(), vd.y() - vo.y(), vd.z() - vo.z();
    vec << P.x() - vo.x(), P.y() - vo.y(), P.z() - vo.z();

    dir = side.cross(vec);

}
static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    // solution1 : vector operation
    // Vector3f P, A, B, C;
    // P << x+0.5, y+0.5, _v[0].z();
    // A << _v[0].x(), _v[0].y(), _v[0].z();
    // B << _v[1].x(), _v[1].y(), _v[1].z();
    // C << _v[2].x(), _v[2].y(), _v[2].z();

    // Vector3f dir1, dir2, dir3;
    // crossProduct(P, A, B, dir1);
    // crossProduct(P, B, C, dir2);
    // crossProduct(P, C, A, dir3);
    // if(dir1.dot(dir2) > 0 && dir2.dot(dir3) > 0)
    // {
    //     return true;
    // }
    // else{
    //     return false;
    // }

    // solution1.1 : vector operation 
    Vector2f P, A, B, C;
    P << x+0.5, y+0.5;
    A << _v[0].x(), _v[0].y();
    B << _v[1].x(), _v[1].y();
    C << _v[2].x(), _v[2].y();
    // get z axis value
    float z1 = zAxisValue(P, A, B); 
    float z2 = zAxisValue(P, B, C);
    float z3 = zAxisValue(P, C, A);

    if((z1>0 && z2>0 && z3>0) || (z1<0&&z2<0&&z3<0))
    {
        return true;
    }
    else
    {
        return false;
    }

    // solution2 : area corase
    // Vector2f P, A, B, C;
    // P << x+0.5, y+0.5;
    // A << _v[0].x(), _v[0].y();
    // B << _v[1].x(), _v[1].y();
    // C << _v[2].x(), _v[2].y();

    // float s = HeronArea(A, B, C);
    // float s1 = HeronArea(P, B, C);
    // float s2 = HeronArea(A, P, C);
    // float s3 = HeronArea(A, B, P);

    // if(s1 + s2 + s3 > s)
    // {
    //     return false;
    // }
    // else
    // {
    //     return true;
    // }
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
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
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
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
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float x_min = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    float x_max = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    float y_min = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    float y_max = std::max(std::max(v[0].y(), v[1].y()), v[2].y());

    int box_lbx = std::max(static_cast<int>(x_min), 0);
    int box_lby = std::max(static_cast<int>(y_min), 0);
    int box_rtx = std::min(static_cast<int>(x_max), width);
    int box_rty = std::min(static_cast<int>(y_max), height);
    for(int x=box_lbx; x<=box_rtx;x++)
    {
        for(int y=box_lby; y <=box_rty; y++)
        {
            if(insideTriangle(x, y, t.v))
            {
                // auto[alpha, beta, gamma] = computeBarycentric2D(x+0.25, y+0.25, t.v);
                // float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                // float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                // z_interpolated *= w_reciprocal;
                // if(depth_buf[get_index(x, y)] > z_interpolated)
                // {
                //     Eigen::Vector3f point;
                //     point.x() = x;
                //     point.y() = y;
                //     point.z() = z_interpolated;
                //     depth_buf[get_index(x, y)] = z_interpolated;
                //     set_pixel(point, t.getColor());
                // }

                // super sampling, min_depth, count of color
                Vector2f pixel[4];
                pixel[0] << x+0.25, y+0.25;
                pixel[1] << x+0.25, y+0.75;
                pixel[2] << x+0.75, y+0.25;
                pixel[3] << x+0.75, y+0.75;

                float z_min = Infinity;
                int count = 0;

                for(int index = 0; index < 4; index++)
                {
                    auto[alpha, beta, gamma] = computeBarycentric2D(pixel[index].x(), pixel[index].y(), t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    
                    if(depth_buf[get_index(x, y)] > z_interpolated)
                    {
                        count++;
                        z_min = std::min(z_interpolated, z_min);
                    }

                }

                if(count > 0)
                {
                    Eigen::Vector3f point;
                    point << x, y, z_min;
                    depth_buf[get_index(x, y)] = z_min;
                    set_pixel(point, t.getColor() * count / 4.0);
                }
            }
        }
    }
    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on