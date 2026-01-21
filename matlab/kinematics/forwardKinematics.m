function T = forwardKinematics(r1, r2, t1, t2)
    % Calculate end-effector position
    px = r1 * cos(t1) + r2 * cos(t1 + t2);
    py = r1 * sin(t1) + r2 * sin(t1 + t2);

    % Construct transformation matrix T to include end-effector position
    T = [1, 0, 0, px;
         0, 1, 0, py;
         0, 0, 1, 0;
         0, 0, 0, 1];
end

