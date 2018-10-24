#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <vector>

using namespace cv;

int i = 0;
int j = 0;
Eigen::MatrixXf M(3, 4);
Eigen::MatrixXf M1(3, 4);

void callbackFuncImg (int event, int x, int y, int, void*)
{
    if (i > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M(0, i) = y;
        M(1, i) = x;
        M(2, i) = 1;
        i += 1;
    }
}

void callbackFuncImg1 (int event, int x, int y, int, void*)
{
    if (j > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M1(0, j) = y;
        M1(1, j) = x;
        M1(2, j) = 1;
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
    Mat new_image (height, width, CV_8UC3);

    namedWindow(argv[1],  WINDOW_AUTOSIZE);
    setMouseCallback(argv[1], callbackFuncImg, NULL);
    imshow(argv[1], image);
    waitKey(0);

    setMouseCallback(argv[1], callbackFuncImg1, NULL);
    imshow(argv[1], image);
    waitKey(0);

    //M << -3,3,1,-1,-1,-1,1,1,1,1,1,1;
    //M1 << -2,2,2,-2,-1,-1,1,1,1,1,1,1;
    M << 1,0,0,1,0,1,0,1,0,0,1,1;
    M1 << 1,3,0,7,2,2,1,11,3,1,1,10;
    std::cout << "Matrix M\n\n" << M << std::endl;
    std::cout << "Matrix M1\n\n" << M1 << std::endl;
    
    Eigen::Vector3f D = M.rightCols<1>();
    Eigen::Vector3f D1 = M1.rightCols<1>();
    std::cout << D << std::endl << std::endl;
    std::cout << D1 << std::endl << std::endl;
    Eigen::Vector3f lambda = (M.leftCols<3>().inverse()) * D;
    Eigen::Vector3f lambda1 = (M1.leftCols<3>().inverse()) * D1;
    
    std::cout << "lambda1:\n" << lambda << std::endl;
    std::cout << "lambda2:\n" << lambda1 << std::endl;

    Eigen::MatrixXf P1(3,3);
    Eigen::MatrixXf P2(3,3);
    P1 = (M.leftCols<3>()).array().rowwise() * lambda.transpose().array();
    P2 = (M1.leftCols<3>()).array().rowwise() * lambda1.transpose().array();
    std::cout << P1 << std::endl;
    std::cout << P2 << std::endl;
    
    Eigen::MatrixXf P(3,3);
    Eigen::MatrixXf P_inverse(3,3);
    P = P2 * P1.inverse();
    std::cout << "P\n" << std::endl << P << std::endl;
    P_inverse = P.inverse();
    std::cout << "P_inverse\n" << std::endl << P_inverse << std::endl;
    
    for (unsigned i = 0; i <= height; ++i) {
        for (unsigned j = 0; j <= width; ++j) {
            Eigen::Vector3f m1_pixel;
            m1_pixel << i, j, 1;
            Eigen::Vector3f m_pixel = P_inverse * m1_pixel;
            
            m_pixel /= m_pixel[2];
            
            int x = std::round(m_pixel[0]);
            int y = std::round(m_pixel[1]);
            std::cout << m_pixel[0] << " " << x << std::endl;
            std::cout << m_pixel[1] << " " << y << std::endl;
            if (x >= 0 and y >= 0 and x < (int)width and y < (int)height) {
                Vec3b color = image.at<Vec3b>(Point(x, y));
                new_image.at<Vec3b>(Point(i, j)) = color;
            }
        }
    }
    
    namedWindow("New image",  WINDOW_AUTOSIZE);
    imshow("New image", new_image);
    waitKey(0);

    imwrite("./new_image.png", new_image);
    
    return 0;
}