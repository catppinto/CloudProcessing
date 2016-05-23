#include "PosOriRepresentation.h"

PosOriRepresentation::PosOriRepresentation()
{
    //ctor
}

PosOriRepresentation::~PosOriRepresentation()
{
    //dtor
}


Eigen::MatrixXd PosOriRepresentation::RotationFromZYX(std::vector<double> zyx_eulerAngles)
{
    double alpha = zyx_eulerAngles[0];
    double beta = zyx_eulerAngles[1];
    double gamma = zyx_eulerAngles[2];

    double c_a = cos(alpha); double s_a = sin(alpha);
    double c_b = cos(beta); double s_b = sin(beta);
    double c_g = cos(gamma); double s_g = sin(gamma);

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  c_a*c_b     , c_a*s_b*s_g-s_a*c_g   , c_a*s_b*c_g+s_a*s_g,
                        s_a*c_b     , s_a*s_b*s_g+c_a*c_g   , s_a*s_b*c_y-c_a*s_g,
                        -s_b        , c_b*s_g               , c_b*c_g;

    return rotation_matrix;
}

Eigen::MatrixXd PosOriRepresentation::RotationFromAngleAxis(std::vector<double> angle_axis)
{
    double theta = angle_axis[0];
    double wx = angle_axis[1];
    double wy = angle_axis[2];
    double wz = angle_axis[3];

    double v_theta = 1-cos(theta) ; double c_theta = cos(theta); double s_theta = sin(theta);

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  wx*wx*v_theta+c_theta       , wx*wy*v_theta-wz*s_theta  , wx*wz*v_theta+wy*s_theta,
                        wx*wy*v_theta+wz*s_theta    , wy*wy*v_theta+c_theta     , wy*wz*v_theta-wx*s_theta,
                        wx*wz*v_theta-wy*s_theta    , wy*wz*v_theta+wx*s_theta  , wz*wz*v_theta+c_theta;

    return rotation_matrix;
}

Eigen::MatrixXd PosOriRepresentation::RotationFromUnitQuaternion(std::vector<double> q)
{
    double e0 = q[0];
    double e1 = q[1];
    double e2 = q[2];
    double e3 = q[3];

    Eigen::MatrixXd rotation_matrix(3,3);
    rotation_matrix <<  1-2*(e2*e2+e3*e3) , 2*(e1*e2-e0*e3)     , 2*(e1*e3+e0*e2),
                        2*(e1*e2+e0*e3)   , 1-2*(e1*e1+e3*e3)   , 2*(e2*e3-e0*e1),
                        2*(e1*e3-e0*e2)   , 2*(e2*e3+e0*e1)     , 1-2*(e1*e1+e2*e2);

    return rotation_matrix;
}

std::vector<double> PosOriRepresentation::ZYXfromRotationMatrix(Eigen::MatrixXd r)
{
    double r11 = r(0,0); double r12 = r(0,1); double r12 = r(0,2);
    double r21 = r(1,0); double r22 = r(1,1); double r22 = r(1,2);
    double r31 = r(2,0); double r32 = r(2,1); double r32 = r(2,2);

    double beta = atan2(-r31, sqrt(r11*r11+r21*r21));
    double alpha = atan2(r21/cos(beta), r11/cos(beta));
    double gamma = atan2(r32/cos(beta), r33/cos(beta));

    std::vector<double> zyx;
    zyx.push_back(alpha);
    zyx.push_back(beta);
    zyx.push_back(gamma);

    return zyx;
}

std::vector<double> PosOriRepresentation::AngleAxisfromRotationMatrix(Eigen::MatrixXd r)
{
    double r11 = r(0,0); double r12 = r(0,1); double r12 = r(0,2);
    double r21 = r(1,0); double r22 = r(1,1); double r22 = r(1,2);
    double r31 = r(2,0); double r32 = r(2,1); double r32 = r(2,2);

    double theta = acos((r11+r22+r33-1)/2);
    double wx = 1/(2*sin(theta))*(r32-r23);
    double wy = 1/(2*sin(theta))*(r13-r31);
    double wz = 1/(2*sin(theta))*(r21-r12);

    std::vector<double> angle_axis;
    angle_axis.push_back(theta);
    angle_axis.push_back(wx);
    angle_axis.push_back(wy);
    angle_axis.push_back(wz);
    return angle_axis;
}

std::vector<double> PosOriRepresentation::UnitQuaternionfromRotationMatrix(Eigen::MatrixXd r)
{
    double r11 = r(0,0); double r12 = r(0,1); double r12 = r(0,2);
    double r21 = r(1,0); double r22 = r(1,1); double r22 = r(1,2);
    double r31 = r(2,0); double r32 = r(2,1); double r32 = r(2,2);

    double e0 = 1/2*sqrt(1+r11+r22+r33);
    double e1 = (r32-r23)/(4*e0);
    double e2 = (r13-r31)/(4*e0);
    double e3 = (r21-r12)/(4*e0);

    std::vector<double> q;
    q.push_back(e0);
    q.push_back(e1);
    q.push_back(e2);
    q.push_back(e3);
   return q;
}

std::vector<double> PosOriRepresentation::UnitQuaternionfromAngleAxis(std::vector<double> angle_axis)
{
    double theta = angle_axis[0];
    double wx = angle_axis[1];
    double wy = angle_axis[2];
    double wz = angle_axis[3];

    double e0 = cos(theta/2);
    double e1 = wx*sin(theta/2);
    double e2 = wy*sin(theta/2);
    double e3 = wz*sin(theta/2);

    std::vector<double> q;
    q.push_back(e0);
    q.push_back(e1);
    q.push_back(e2);
    q.push_back(e3);
    return q;
}

std::vector<double> PosOriRepresentation::AngleAxisfromUnitQuaternion(std::vector<double> q)
{
    double e0 = q[0];
    double e1 = q[1];
    double e2 = q[2];
    double e3 = q[3];

    double theta = 2*acos(e0);
    double wx = e1/(sin(theta/2));
    double wy = e2/(sin(theta/2));
    double wz = e3/(sin(theta/2));

    std::vector<double> angle_axis;
    angle_axis.push_back(theta);
    angle_axis.push_back(wx);
    angle_axis.push_back(wy);
    angle_axis.push_back(wz);
    return angle_axis;
}
