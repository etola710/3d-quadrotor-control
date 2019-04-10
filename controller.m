function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================
Kd1=10;
Kd2=10;
Kd3=10;
Kp1=30;
Kp2=30;
Kp3=30;
Kd_phi=20;
Kd_th=20;
Kd_psi=20;
Kp_phi=300;
Kp_th=300;
Kp_psi=300;
%given desired acc,vel,pos calculate required acc
r_des_dd1 = des_state.acc(1) + Kd1*(des_state.vel(1)-state.vel(1))+ Kp1*(des_state.pos(1)-state.pos(1));
r_des_dd2 = des_state.acc(2) + Kd2*(des_state.vel(2)-state.vel(2))+ Kp2*(des_state.pos(2)-state.pos(2));
r_des_dd3 = des_state.acc(3) + Kd3*(des_state.vel(3)-state.vel(3))+ Kp3*(des_state.pos(3)-state.pos(3));
%convert required acc to find roll/pitch angles
phi_des = (1/params.gravity)*(r_des_dd1*sin(des_state.yaw)- r_des_dd2*cos(des_state.yaw));
theta_des = (1/params.gravity)*(r_des_dd1*cos(des_state.yaw)+ r_des_dd2*sin(des_state.yaw));
%use to find required control inputs
%u1
u1 = params.mass*params.gravity + params.gravity*r_des_dd3;
%u2
%p_des = q_des =  0 omegas
u2 = [Kp_phi*(phi_des - state.rot(1)) + Kd_phi*(-state.omega(1));
    Kp_th*(theta_des - state.rot(2)) + Kd_th*(-state.omega(2));
    Kp_psi*(des_state.yaw - state.rot(3)) + Kd_psi*(des_state.yawdot-state.omega(3))];
% Thrust
%u1 = sum(F)
F = u1;
% Moment
%u2 = I*u2
M = params.I*u2;
% =================== Your code ends here ===================

end
