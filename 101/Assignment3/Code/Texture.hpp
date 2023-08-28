//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
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
        auto v_img = (1.0 - v) * height;
        try 
        {
            auto color = image_data.at<cv::Vec3b>(static_cast<int>(v_img), static_cast<int>(u_img));
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
        catch (const std::exception& e)
        {
            std::cout << u_img << " " << v_img << std::endl;
            return Eigen::Vector3f(0.0, 0.0, 0.0);
        }
        
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        int u0 = u * width;
        int u1 = u0 + 1;
        float s = u * width - u0;
        int v0 = (1.0 - v) * height;
        int v1 = v0 + 1;
        float t = (1.0 - v) * height - v0;

        try 
        {
            auto c00 = image_data.at<cv::Vec3b>(v0, u0);
            auto c10 = image_data.at<cv::Vec3b>(v1, u0);
            auto c01 = image_data.at<cv::Vec3b>(v0, u1);
            auto c11 = image_data.at<cv::Vec3b>(v1, u1);
            
            auto c0 = c00 + s * (c10 - c00);
            auto c1 = c01 + s * (c11 - c01);
            auto color = c0 + t * (c1 - c0);
            return Eigen::Vector3f(color[0], color[1], color[2]);
        }
        catch (const std::exception& e)
        {
            std::cout << u0 << " " << v0 << std::endl;
            return Eigen::Vector3f(0.0, 0.0, 0.0);
        }
        
    }

};
#endif //RASTERIZER_TEXTURE_H
