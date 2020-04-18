#ifndef QUAT_2_DCM_HPP
#define QUAT_2_DCM_HPP

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>

// m shoubld be be a 3 * 3 matrix
double quat_2_dcm(geometry_msgs::Quaternion q, double m[][3]){
    double sqw = q.w*q.w;
    double sqx = q.x*q.x;
    double sqy = q.y*q.y;
    double sqz = q.z*q.z;

    // invs (inverse square length) is only required if quaternion is not already normalised
    double invs = 1 / (sqx + sqy + sqz + sqw);
    m[0][0] = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
    m[1][1] = (-sqx + sqy - sqz + sqw)*invs ;
    m[2][2] = (-sqx - sqy + sqz + sqw)*invs ;

    double tmp1 = q.x*q.y;
    double tmp2 = q.z*q.w;
    m[1][0] = 2.0 * (tmp1 + tmp2)*invs ;
    m[0][1] = 2.0 * (tmp1 - tmp2)*invs ;

    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    m[2][0] = 2.0 * (tmp1 - tmp2)*invs ;
    m[0][2] = 2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    m[2][1] = 2.0 * (tmp1 + tmp2)*invs ;
    m[1][2] = 2.0 * (tmp1 - tmp2)*invs ;

}

geometry_msgs::Point32 xyz_rotation_dcm(geometry_msgs::Point32 X, double m[][3]) {
    geometry_msgs::Point32 X_out;

    X_out.x = m[0][0] * X.x + m[0][1] * X.y + m[0][2] * X.z;
    X_out.y = m[1][0] * X.x + m[1][1] * X.y + m[1][2] * X.z;
    X_out.z = m[2][0] * X.x + m[2][1] * X.y + m[2][2] * X.z;

    return X_out;
}



#endif // QUAT_2_DCM_HPP
