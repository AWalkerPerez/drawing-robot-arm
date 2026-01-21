close all
clear
clc

% Parameters for robot arm
r1 = 1.0;    % Length of first link
r2 = 1.0;    % Length of second link
t1 = 0.5;    % Initial angle of joint 1
t2 = 0.5;    % Initial angle of joint 2

% Desired position
x_d = [1.0; 1.0]; % Desired x and y coordinates of the end-effector

% PID controller gains
Kp = 1e-6;
Ki = 0.01;
Kd = 0.05;

% Initialize errors
prev_error = [0; 0];
integral_error = [0; 0];

% Time settings
dt = 0.01;       % Time step
max_iterations = 10000; % Maximum number of iterations
tolerance = 1e-3; % Position error tolerance

time_var=0;
desired_trajectory_x=1;
desired_trajectory_y=1;

[t1, t2] = ik_differential(r1, r2, t1, t2, [desired_trajectory_x desired_trajectory_y])
t1=0;
t2=pi/2;

for i = 1:max_iterations
    % Current end-effector position
    T = forwardKinematics(r1, r2, t1, t2);
    x = T(1, 4);
    y = T(2, 4);
    current_position = [x; y]
    
    time_var=dt*i;
    desired_trajectory_x=0.30*cos(0.4*time_var+0.3)+1;
    desired_trajectory_y=0.30*sin(0.1*time_var)+1;


    x_d = [desired_trajectory_x; desired_trajectory_y];

    % Calculate position error
    %error = x_d - current_position;
    
    % Break loop if within tolerance
%     if norm(error) < tolerance
%         disp('Desired position reached!');
%         break;
%     end
%     
    % PID control terms
    %integral_error = integral_error + error * dt;
    %derivative_error = (error - prev_error) / dt;
    %control_signal = Kp * error + Ki * integral_error + Kd * derivative_error;
    
    % Adjust joint angles using inverse kinematics and control signal
    [new_t1, new_t2] = ik_differential(r1, r2, t1, t2, x_d);
    
    % Update joint angles
    t1 = new_t1;
    t2 = new_t2;
    
    % Update previous error
    prev_error = error;
    
    % Optional: Plot current position for visualization
    if mod(i,200)==0
          plot([0, r1*cos(t1), r1*cos(t1) + r2*cos(t1 + t2)], ...
             [0, r1*sin(t1), r1*sin(t1) + r2*sin(t1 + t2)], '-o');
            hold on
            axis equal;
            xlim([-2, 2]);
          ylim([-2, 2]);
             pause(0.01);

    end
end

% Display final joint angles
disp(['Final joint angles: t1 = ', num2str(t1), ', t2 = ', num2str(t2)]);
disp(['Final end-effector position: [', num2str(current_position(1)), ', ', num2str(current_position(2)), ']']);
