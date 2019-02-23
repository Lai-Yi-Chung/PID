#include "geometry_control.h"

geometry_control::geometry_control()
{
    //q[4] = {0.0, 0.0, 0.0, 0.0};
    sample_freq = 100;
    inertia.setZero(3,3);
    K_R.setZero(3);K_omega.setZero(3);K_i.setZero(3);K_m.setZero(3);

    F_des.setZero(3);Global_yaw_des.setZero(3);
    B_X_des.setZero(3);B_Y_des.setZero(3);B_Z_des.setZero(3);

    R.setZero(3,3);R_des.setZero(3,3);
    last_R_des.setZero(3,3);R_des_dot.setZero(3,3);
    omega.setZero(3);omega_des.setZero(3);
    last_omega_des.setZero(3);omega_des_dot.setZero(3);

    error_R.setZero(3); error_omega.setZero(3); temp.setZero(3);
    accum_attitude_error.setZero(3);

    state.setZero(4);
    force.setZero(4);
    
    inertia << 0.082 ,      0,      0,
                   0 , 0.0845,      0,
                   0 ,      0, 0.1377;

    K_R << 240 , 240 , 300;
    K_omega << 90 , 90 , 150;
    K_i << 3, 3, 3;
    K_m << 13, 13, 13;


}
Eigen::VectorXd geometry_control::vee_map(Eigen::MatrixXd matrix){
    Eigen::VectorXd vector;
    vector.setZero(3);
    vector(0) = matrix(2,1);
    vector(1) = matrix(0,2);
    vector(2) = matrix(1,0);
    return vector;
}

Eigen::MatrixXd geometry_control::inv_vee_map(Eigen::VectorXd vector){
    Eigen::MatrixXd matrix;
    matrix.setZero(3,3);
    matrix(0,0) =  0;
    matrix(0,1) = -vector(2);
    matrix(0,2) =  vector(1);
    matrix(1,0) =  vector(2);
    matrix(1,1) =  0;
    matrix(1,2) = -vector(0);
    matrix(2,0) = -vector(1);
    matrix(2,1) =  vector(0);
    matrix(2,2) =  0;
    return matrix;
}
void geometry_control::set(float F_x, float F_y, float F_z){

    Global_yaw_des << 1, 0, 0;

    F_des(0) = F_x;
    F_des(1) = F_y;
    F_des(2) = F_z;
    state(0) = sqrt(F_x*F_x + F_y*F_y + F_z*F_z);

    B_Z_des = F_des / F_des.norm();
    B_Y_des = Global_yaw_des / Global_yaw_des.norm();
    Vector3d a1,a2,a3;
    for(int i = 0 ; i < 3 ; i++){
        a1(i) = B_Z_des(i);
        a2(i) = B_Y_des(i);
    }
    a3 = a1.cross(a2);
    for(int i = 0 ; i < 3 ; i++)
        B_X_des(i) = a3(i);

    // Get R_des
    for(int i = 0 ; i < 3 ; i++){
        R_des(0+i*3) = B_X_des(i);
        R_des(1+i*3) = B_Y_des(i);
        R_des(2+i*3) = B_Z_des(i);
    }
    // Get R_des_dot
    R_des_dot = (R_des - last_R_des)*(1/sample_freq);
    last_R_des = R_des;

    // Get angular velocity hat matrix
    omega_des = vee_map( R_des.transpose()*R_des_dot );

    // Get angular_velocity_des_dot
    omega_des_dot = (omega_des - last_omega_des)*(1/sample_freq);
    omega_des = last_omega_des;

}


Eigen::VectorXd geometry_control::geometry_controller(Eigen::VectorXd q , Eigen::VectorXd F ,Eigen::VectorXd ang_vel){
    for(int i = 0; i < 3 ; i++)
        omega(i) = ang_vel(i);
    Quaternion_to_Rotation_Matrix(q(0),q(1),q(2),q(3));
    set(F(0),F(1),F(2));
    error_R = vee_map(R_des.transpose()*R - R.transpose()*R_des)/2;
    error_omega = omega - R.transpose()*R_des*omega_des;
    for(int i = 0 ; i < 3 ;i++){
        const float c = 2, d = 1, limit = 6;
        accum_attitude_error(i) += (d*error_omega(i) + c*error_R(i))*sample_freq;
        //dead zone
        if(accum_attitude_error(i) > limit)
            accum_attitude_error(i) = limit;
        else if(accum_attitude_error(i) < -limit)
            accum_attitude_error(i) = -limit;
    }
    temp = R.transpose()*R_des*omega_des*inertia*R.transpose()*R_des*omega_des;
    temp += inertia*R.transpose()*R_des*omega_des_dot;

    for(int i = 0 ; i < 3 ; i++)
        state(i+1) = - K_R(i)*error_R(i) - K_omega(i)*error_omega(i) + K_i(i)*accum_attitude_error(i) - K_m(i)*temp(i);

    //Mixing matrix
    force(0) =  state(3) - state(2) + state(1) + state(0);
    force(1) = -state(3) - state(2) - state(1) + state(0);
    force(2) =  state(3) + state(2) - state(1) + state(0);
    force(3) = -state(3) + state(2) + state(1) + state(0);
    
    return  force;
}


void geometry_control::Quaternion_to_Rotation_Matrix(float q0, float q1, float q2, float q3){
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;
    float q1q1_q2q2 = q1q1 + q2q2;

    R(0) = q0q0 + q1q1 - q2q2 - q3q3;
    R(1) = 2*(q1q2 + q0q3);
    R(2) = 2*(q1q3 - q0q2);
    R(3) = 2*(q1q2 - q0q3);
    R(4) = q0q0 - q1q1 + q2q2 - q3q3;
    R(5) = 2*(q2q3 + q0q1);
    R(6) = 2*(q1q3 + q0q2);
    R(7) = 2*(q2q3 - q0q1);
    R(8) = q0q0 - q1q1 - q2q2 + q3q3;

}

