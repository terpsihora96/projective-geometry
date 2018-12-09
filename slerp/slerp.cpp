#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Eigenvalues>
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#define TIMER_ID 0
#define TIMER_INTERVAL 20

int animation_ongoing;

double t1 = 2;      // overall time of the animation
double t = 0;       // time elapsed

Eigen::Vector3f C1;     // position of the object in the begining
Eigen::Vector3f C2;     // position of the object in the end
Eigen::MatrixXf E(3, 3);

double phi_1, theta_1, psi_1;
double phi_2, theta_2, psi_2;

void on_keyboard(unsigned char key, int, int);
void on_reshape(int width, int height);
void on_timer(int id);
void display(void);

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

Eigen::MatrixXf Rodrigues(Eigen::Vector3f p, double angle, Eigen::MatrixXf E)
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
            phi = atan2(A(2, 1), A(2, 2));
        }
        else {
            psi = atan2(A(0, 1), A(1, 1));
            theta = M_PI / 2; 
            phi = 0;
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

Eigen::Vector4f A2AngleAxis(Eigen::MatrixXf A, Eigen::MatrixXf E)
{
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

    double angle = acos((u.dot(u1)) / (u.norm() * u1.norm()));

    Eigen::Vector4f v;
    v << p, angle;

    return v;
}

Eigen::Vector4f AngleAxis2Q(Eigen::Vector3f p, double angle)
{
    Eigen::Vector4f q;
    q << sin(angle / 2.0) * p.normalized(), cos(angle / 2.0);

    return q;
}

Eigen::Vector4f Q2AngleAxis(Eigen::Vector4f q)
{
    q /= q.norm();
    if (q(3) < 0) {
        q =-q;
    }

    Eigen::Vector3f p;

    if (abs(q(3)) == 1) {
        p << 1, 0, 0;
    }
    else {
        p << q(0), q(1), q(2);
        p /= p.norm();
    }

    Eigen::Vector4f v;
    v << p, 2 * acos(q(3));

    return v;
}

Eigen::Vector4f slerp(Eigen::Vector4f q1, Eigen::Vector4f q2)
{
    double cos_ = q1.dot(q2) / (q1.norm() * q2.norm());

    if (cos_ < 0) {
        q1 = -q1;
        cos_ = -cos_;
    }

    if (cos_ > 0.95) {
        return q1;
    }
    
    double angle = acos(cos_);
    Eigen::Vector4f q;
    q = (sin(angle * (1 - t / t1)) * q1 + sin(angle * (t / t1)) * q2) / sin(angle);
    
    return q;
}

static void draw_coosys()
{
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3f(1, 0, 0);
        glVertex3f(0, 0, 0);

        glColor3f(0, 1, 0);
        glVertex3f(0, 1, 0);
        glVertex3f(0, 0, 0);

        glColor3f(0, 0, 1);
        glVertex3f(0, 0, 1);
        glVertex3f(0, 0, 0);
    glEnd();
    glEnable(GL_LIGHTING);
}

int main(int argc, char** argv)
{
    E << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    phi_1 = 7 * M_PI / 6;
    theta_1 = M_PI / 4;
    psi_1 = 3 * M_PI / 4;
    
    phi_2 = M_PI/2;
    theta_2 = -M_PI/3;
    psi_2 = 5 * M_PI/4;

    C1 << 2, 2, 2;
    C2 << 7, 3, 0; 
 
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    
    glutInitWindowSize(700, 700);
    glutInitWindowPosition(300, 300);
    glutCreateWindow("SLerp");
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClearDepth(1.0f);
    
    animation_ongoing = 0;
    
    glutKeyboardFunc(on_keyboard);
    glutReshapeFunc(on_reshape);
    glutDisplayFunc(display);
    glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
    glEnable(GL_DEPTH_TEST);
    
    glutMainLoop();

    return 0;
}


void display(void)
{
    // Coeffs for lighting
    GLfloat light_position[] = { 1, 1, 1, 0 };
    GLfloat light_ambient[] = { 0.1, 0.1, 0.1, 1 };
    GLfloat light_diffuse[] = { 0.7, 0.7, 0.7, 1 };
    GLfloat light_specular[] = { 0.9, 0.9, 0.9, 1 };
    GLfloat ambient_coeffs[] = { 0.3, 0.3, 0.4, 1 };
    GLfloat diffuse_coeffs[] = { 0.6, 0.6, 0.85, 1 };
    GLfloat specular_coeffs[] = { 0.6, 0.6, 0.6, 1 };
    GLfloat shininess = 60;

    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);

    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_coeffs);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_coeffs);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular_coeffs);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);

    glEnable(GL_COLOR_MATERIAL);
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(  12, 2, 12, 
                6, 2, 3, 
                0, 1, 0  );
    
    glPushMatrix();
        glTranslatef(C1(0), C1(1), C1(2));
        glRotatef(phi_1, 1, 0, 0);
        glRotatef(theta_1, 0, 1, 0); 
        glRotatef(psi_1, 0, 0, 1);
        draw_coosys();
    glPopMatrix();
    
    Eigen::MatrixXf A1(3, 3);
    Eigen::MatrixXf A2(3, 3);
    A1 = Euler2A(phi_1 * M_PI / 180.0, theta_1 * M_PI / 180.0, psi_1 * M_PI / 180.0);
    A2 = Euler2A(phi_2 * M_PI / 180.0, theta_2 * M_PI / 180.0, psi_2 * M_PI / 180.0);

    Eigen::Vector4f p_angle1;
    p_angle1 = A2AngleAxis(A1, E);
    double angle1 = p_angle1(3);
    Eigen::Vector3f p1(p_angle1(0), p_angle1(1), p_angle1(2));
    
    Eigen::Vector4f p_angle2;
    p_angle2 = A2AngleAxis(A2, E);
    double angle2 = p_angle2(3);
    Eigen::Vector3f p2(p_angle2(0), p_angle2(1), p_angle2(2));

    Eigen::Vector4f q1 = AngleAxis2Q(p1, angle1);
    Eigen::Vector4f q2 = AngleAxis2Q(p2, angle2);

    Eigen::Vector4f q = slerp(q1, q2);
    
    Eigen::Vector4f p_angle = Q2AngleAxis(q);
    
    Eigen::Vector3f p(p_angle(0), p_angle(1), p_angle(2));
    Eigen::MatrixXf A = Rodrigues(p, p_angle(3), E);
    
    Eigen::Vector3f angles = A2Euler(A);
    
    Eigen::Vector3f c_t;
    c_t = (1 - t / t1) * C1 + t / t1 * C2;
    
    glPushMatrix();
        glTranslatef(c_t(0), c_t(1), c_t(2));
        glRotatef(angles(0) * 180 / M_PI, 1, 0, 0);
        glRotatef(angles(1) * 180 / M_PI, 0, 1, 0); 
        glRotatef(angles(2) * 180 / M_PI, 0, 0, 1);
        glutWireDodecahedron();
    glPopMatrix();

    
    glPushMatrix();
        glTranslatef(C2(0), C2(1), C2(2));
        glRotatef(phi_2, 1, 0, 0);
        glRotatef(theta_2, 0, 1, 0); 
        glRotatef(psi_2, 0, 0, 1);
        draw_coosys();
    glPopMatrix();

    glutSwapBuffers();
}

void on_timer(int id)
{
    if (id != TIMER_ID) { 
        return;
    }
    
    if (t < t1) {
        t += 0.02;
    }
    else {
        animation_ongoing = 0;
    }

    glutPostRedisplay();
    if (animation_ongoing) {
        glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
    }
}

void on_keyboard(unsigned char key, int, int)
{
    // Exit on Esc
    switch (key) {
    case 27:
        exit(0);
        break;

    // Go
    case 'g':
    case 'G':
        if (!animation_ongoing) {
            glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
            animation_ongoing = 1;
        }
        break;

    // Stop
    case 's':
    case 'S':
        animation_ongoing = 0;
        break;
    
    // Reset
    case 'r':
    case 'R':
        t = 0;
        if (!animation_ongoing) {
            glutTimerFunc(TIMER_INTERVAL, on_timer, TIMER_ID);
            animation_ongoing = 1;
        }
        break;
    }
}

void on_reshape(int width, int height)
{
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(45, (float) width / height, 1, 100);
}
