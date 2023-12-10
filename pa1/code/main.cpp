#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>
// #include <math.h>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    float al = angle * MY_PI / 180.0f;

    float cosa = std::cos(al);
    float fcosa = 1 - std::cos(al);

    float x = axis[0], y = axis[1], z = axis[2];
    float l = std::sqrt(x*x+y*y+z*z);
    // 归一化轴
    x /= l, y /= l, z /= l;
    float xx = x*x, yy = y*y, zz = z*z;

    model <<    cosa + fcosa * xx, x * y * fcosa - std::sin(al) * z, x * z * fcosa + std::sin(al) * y, 0,
                x * y * fcosa + std::sin(al) * z, cosa + yy * fcosa, y * z * fcosa - std::sin(al) * x, 0,
                x * z * fcosa - std::sin(al) * y, y * z * fcosa + x * std::sin(al), cosa + zz * fcosa, 0,
                0, 0, 0, 1;
    return model;

}


Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    double al = rotation_angle * MY_PI / 180.0f;

    model <<    std::cos(al),    -1.f * std::sin(al), 0, 0,
                std::sin(al),    std::cos(al),        0, 0,
                0,               0,                   1, 0,
                0,               0,                   0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float n = -zNear, f = -zFar;
    float half_al = eye_fov * MY_PI / 180.0f / 2.f;
    float t = fabs(n)*std::tan(half_al), b = -t;
    float r = aspect_ratio * t, l = -r;

    Eigen::Matrix4f scale, transmit, pres;
    transmit<<  1, 0, 0, -(r+l) / 2.0f,
                0, 1, 0, -(t+b) / 2.0f,
                0, 0, 1, -(n+f) / 2.0f,
                0, 0, 0, 1;
    scale<<     2.f / (r-l),    0,  0,  0,
                0,  2.f / (t-b),0,  0,
                0,  0,  2.f / (n-f),0,
                0,  0,  0,  1;
    pres<<      n,  0,  0,  0,
                0,  n,  0,  0,
                0,  0,  n+f,-(n*f),
                0,  0,  1,  0;

    projection = scale * transmit * pres * projection;
    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        // r.set_model(get_model_matrix(angle));
        // get_rotation
        Eigen::Vector3f axis = {1,0,0};
        r.set_model(get_rotation(axis, angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
