function [new_t1,new_t2] = ik_differential(r1,r2,t1,t2,x_d)
%forward kinematics for given theta values
T = forwardKinematics(r1,r2,t1,t2);
%get position for end effector
x=T(1,4);
y=T(2,4);
%distance between desired and current position
delta_x=[x_d(1)-x;x_d(2)-y];

%delta_x=[0.001; 0.001]

%delta_x_dot=delta_x/elapsed_t;

%compute jacobian
J=ik_jacobian(r1,r2,t1,t2);
%compute inverse jacobian
inv_J=inv(J);
%compute change of joint angles
delta_q=inv_J*delta_x;
%compute the next joint angles
new_t1=t1+delta_q(1);
new_t2=t2+delta_q(2);
end

