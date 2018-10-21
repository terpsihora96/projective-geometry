#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

int i = 0;
int j = 0;
Eigen::MatrixXf M(4,3);
Eigen::MatrixXf M1(4,3);

void callbackFuncImg (int event, int x, int y, int, void*)
{
    if (i > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M(i, 0) = y;
        M(i, 1) = x;
        M(i, 2) = 1;
        i += 1;
    }
}

void callbackFuncImg1 (int event, int x, int y, int, void*)
{
    if (j > 3) {
        return;
    } else if (event ==  EVENT_LBUTTONDOWN) {
        std::cout << "(" << y << "," << x << ")" << std::endl;
        M1(j, 0) = y;
        M1(j, 1) = x;
        M1(j, 2) = 1;
        j += 1;
    }
}

int main (int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: ./pg image_name" << std::endl;
        return -1;
    }

    Mat image;
    image =  imread(argv[1], CV_LOAD_IMAGE_COLOR);
    unsigned width = image.cols;
    unsigned height = image.rows;    

    if (image.empty()) {
        std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    namedWindow(argv[1],  WINDOW_AUTOSIZE);
    setMouseCallback(argv[1], callbackFuncImg, NULL);
    imshow(argv[1], image);
    waitKey(0);

    Mat new_image =  Mat::ones(height, width, CV_8UC3);
    namedWindow("New image",  WINDOW_AUTOSIZE);
    setMouseCallback("New image", callbackFuncImg1, NULL);
    imshow("New image", new_image);
    waitKey(0);

    M << 1,0,0,0,1,0,0,0,1,1,1,1;
    M1 << 1,2,3,3,2,1,0,1,1,7,11,10;
    std::cout << "Matrix M\n\n" << M << std::endl;
    std::cout << "Matrix M1\n\n" << M1 << std::endl;
    
    Eigen::Vector3f D = M.block<1,3>(3,0).transpose();
    Eigen::Vector3f D1 = M1.block<1,3>(3,0).transpose();

    Eigen::Vector3f lambda = (M.block<3,3>(0,0)).inverse() * (D);
    Eigen::Vector3f lambda1 = (M1.block<3,3>(0,0)).inverse() * (D1);
    
    cout << "\nSolution\n";
    cout << "1:\n" << lambda << endl;
    cout << "2:\n" << lambda1 << endl;

    Eigen::MatrixXf P1(3,3);
    P1 = (M.block<3,3>(0,0)).array().colwise() * lambda.array();
    Eigen::MatrixXf P2(3,3);
    P2 = (M1.block<3,3>(0,0)).array().colwise() * lambda1.array();
    
    Eigen::MatrixXf P(3,3);
    Eigen::MatrixXf P_inverse(3,3);
    P = P2 * P1.inverse();
    P_inverse = P.inverse();
    cout << P << endl;
    
    return 0;
}