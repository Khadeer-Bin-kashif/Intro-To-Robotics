function [x, y, z, R] = pincherFK(jointAngles)
% pincherFK: Calculates the Forward Kinematics for the Phantom X Pincher.
%
% Arguments:
%   jointAngles: A 1x4 or 4x1 vector [th1, th2, th3, th4] in RADIANS.
%
% Returns:
%   x, y, z: The Cartesian position of the end-effector in mm.
%   R: The 3x3 Rotation Matrix representing the end-effector orientation.

    % 1. Extracting individual joint angles
    th1 = jointAngles(1);
    th2 = jointAngles(2);
    th3 = jointAngles(3);
    th4 = jointAngles(4);

    % 2. Physical Constants (from DH table in mm)
    a2 = 103; 
    a3 = 102; 
    a4 = 71;
    d1 = 0;

    % 3. DH Transformation Template
    % Using the 'Standard DH' matrix.
    T_dh = @(a, alpha_rad, d, theta) [
        cos(theta), -sin(theta)*cos(alpha_rad),  sin(theta)*sin(alpha_rad), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha_rad), -cos(theta)*sin(alpha_rad), a*sin(theta);
        0,           sin(alpha_rad),             cos(alpha_rad),            d;
        0,           0,                           0,                          1
    ];

    % 4. Computing individual transformations
    T01 = T_dh(0, deg2rad(90), d1, th1);
    T12 = T_dh(a2, deg2rad(180), 0, th2);
    T23 = T_dh(a3, deg2rad(0), 0, th3);
    T34 = T_dh(a4, deg2rad(0), 0, th4);

    % 5. Total Transformation
    T04 = T01 * T12 * T23 * T34;

    % 6. Outputs
    x = T04(1, 4);
    y = T04(2, 4);
    z = T04(3, 4);
    R = T04(1:3, 1:3);
end
