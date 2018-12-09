#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Eigenvalues>

Eigen::MatrixXf Euler2A(double phi, double theta, double psi)
{
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    double cos_phi = cos(phi);
    double sin_phi = sin(phi);
    double cos_psi = cos(psi);
    double sin_psi = sin(psi);

    Eigen::MatrixXf R_x(3, 3);
    R_x << 1, 0, 0, 0, cos_phi, -sin_phi, 0, sin_phi, cos_phi; 
    
    Eigen::MatrixXf R_y(3, 3);
    R_y << cos_theta, 0, sin_theta, 0, 1, 0, -sin_theta, 0, cos_theta; 
    
    Eigen::MatrixXf R_z(3, 3);
    R_z << cos_psi, -sin_psi, 0, sin_psi, cos_psi, 0, 0, 0, 1;

    return R_z * R_y * R_x;
}

Eigen::MatrixXf Rodriguez(Eigen::Vector3f p, double angle, Eigen::MatrixXf E)
{
    Eigen::MatrixXf p_pt(3, 3);
    p_pt = p * p.transpose();
    Eigen::MatrixXf Px(3, 3);
    Px << 0, -p(2), p(1), p(2), 0, -p(0), -p(1), p(0), 0;

    return p_pt + cos(angle) * (E - p_pt) + sin(angle) * Px;
}

Eigen::Vector3f A2Euler(Eigen::MatrixXf A)
{
    Eigen::Vector3f v;
    double phi, theta, psi;

    if (A(2, 0) < 1) {
        if (A(2, 0) > -1) {
            psi = atan2(A(1, 0), A(0, 0));
            theta = asin(-A(2, 0));
            phi = atan2 (A(2, 1), A(2, 2));
        }
        else {
            psi = atan2(A(0, 1), A(1, 1));
            theta = M_PI / 2; 
            phi = 0 ;
        }
    }
    else {
        psi = atan2(-A(0, 1), A(1, 1));
        theta = -M_PI / 2;
        phi = 0;
    }

    v << phi, theta, psi;

    return v;
}

int main()
{
    Eigen::MatrixXf E(3, 3);
    E << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    Eigen::MatrixXf A(3, 3);
    A = Euler2A(-atan(0.25), -asin(0.8888888888888888), atan(4));
    
    Eigen::MatrixXf A1(3, 3);
    A1 = A - E;
    
    Eigen::Vector3f v1(A1.col(0));
    Eigen::Vector3f v2(A1.col(1));
    Eigen::Vector3f p = v1.cross(v2);
    p = p.normalized();
    Eigen::Vector3f u = A1.col(0);
    Eigen::Vector3f u1 = A * u;

    if ((u.cross(u1)).dot(p) < 0) {
        p = -p;
    }

    double angle = acos((u.dot(u1)) / u.norm() * u1.norm());

    Eigen::MatrixXf Rp_angle(3, 3);
    Rp_angle = Rodriguez(p, angle, E);
    std::cout << Rp_angle << std::endl;

    Eigen::Vector3f v;
    v = A2Euler(A);
    std::cout << v << std::endl;
    
    return 0;
}