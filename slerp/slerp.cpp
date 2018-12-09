#include <iostream>
#include <eigen3/Eigen/Eigenvalues>

int main()
{
    /*
    if (argc < 4) {
        exit(0);
    }

    char* theta = argv[1];
    char* phi   = argv[2];
    char* psi   = argv[3];

    std::cout << theta << phi << psi << std::endl;
    */

    Eigen::MatrixXf A(3, 3);
    A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    std::cout << A << std::endl;
    Eigen::MatrixXf A1(3, 3);
    A1 = A - 

    return 0;
}