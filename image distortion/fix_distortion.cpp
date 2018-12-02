#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <vector>

using namespace cv;

Eigen::MatrixXf M(3, 4);
Eigen::MatrixXf M1(3, 4);

void callbackFuncImg (int event, int x, int y, int, void*)
{
    static int i = 0;

    if (i > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M.col(i) << y, x, 1;
        i += 1;
    }
}

void callbackFuncImg1 (int event, int x, int y, int, void*)
{
    static int j = 0;

    if (j > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M1.col(j) << y, x, 1;
        j += 1;
    }
}

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "Usage: ./pg image_name" << std::endl;
        return -1;
    }

    Mat image;
    image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    if (image.empty()) {
        std::cerr << "Failed to open an image." << std::endl;
        return -1;
    }

    unsigned height = image.rows;
    unsigned width = image.cols;
    Mat new_image(height, width, CV_8UC3);

    namedWindow(argv[1],  WINDOW_NORMAL);
    setMouseCallback(argv[1], callbackFuncImg, NULL);
    imshow(argv[1], image);
    waitKey(0);

    setMouseCallback(argv[1], callbackFuncImg1, NULL);
    imshow(argv[1], image);
    waitKey(0);

    std::cout << "\nMatrix M\n" << M << std::endl;
    std::cout << "\nMatrix M1\n" << M1 << std::endl;
    
    Eigen::Vector3f D = M.rightCols<1>();
    Eigen::Vector3f D1 = M1.rightCols<1>();
    
    Eigen::Vector3f lambda = (M.leftCols<3>().inverse()) * D;
    Eigen::Vector3f lambda1 = (M1.leftCols<3>().inverse()) * D1;

    Eigen::MatrixXf P1(3,3);
    Eigen::MatrixXf P2(3,3);
    P1 = (M.leftCols<3>()).array().rowwise() * lambda.transpose().array();
    P2 = (M1.leftCols<3>()).array().rowwise() * lambda1.transpose().array();

    Eigen::MatrixXf P(3,3);
    Eigen::MatrixXf P_inverse(3,3);
    P = P2 * P1.inverse();
    P_inverse = P.inverse();
    
    for (unsigned i = 0; i < height; ++i) {
        for (unsigned j = 0; j < width; ++j) {
            Eigen::Vector3f m1_pixel;
            m1_pixel << i, j, 1;
            Eigen::Vector3f m_pixel = P_inverse * m1_pixel;
            
            m_pixel /= m_pixel[2];
            int i1 = std::round(m_pixel[0]);
            int j1 = std::round(m_pixel[1]);

            if (i1 >= 0 and i1 < (int)height and j1 >= 0 and j1 < (int)width) {
                Vec3b color = image.at<Vec3b>(Point(j1,i1));
                new_image.at<Vec3b>(Point(j,i)) = color;
            }
        }
    }
    
    namedWindow("New image",  WINDOW_NORMAL);
    imshow("New image", new_image);
    waitKey(0);

    imwrite("./new_image.png", new_image);
    
    return 0;
}
