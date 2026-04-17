function [x, y, z, R, theta, phi] = pincherFK(jointAngles)
    % pincherFK Computes the Forward Kinematics for the PhantomX Pincher arm
    % using the Product of Exponentials (PoE) formulation.
    %
    % ARGUMENTS:
    %   jointAngles : A 4-element array [theta1, theta2, theta3, theta4]
    %                 representing the current servo angles.
    %                 UNITS: Radians
    %
    % OUTPUTS:
    %   x, y, z     : The 3D position of the end-effector relative to the base.
    %                 UNITS: Millimeters (mm)
    %   R           : The 3x3 Rotation Matrix representing the 3D orientation 
    %                 of the end-effector frame.
    %   theta       : The base pan angle (yaw) of the robot.
    %                 UNITS: Radians
    %   phi         : The final pitch angle of the gripper relative to the 
    %                 horizontal plane (sum of pitch joints).
    %                 UNITS: Radians

    % 1. Extract joint angles
    th1 = jointAngles(1); % Base Pan
    th2 = jointAngles(2); % Shoulder Pitch
    th3 = jointAngles(3); % Elbow Pitch
    th4 = jointAngles(4); % Wrist Pitch

    % Calculate total pitch angle (phi) and pan angle (theta)
    theta = th1;
    phi = th2 + th3 + th4; 

    % 2. Define Link Lengths (Units: mm)
    L1 = 121.8;
    L2 = 102.38;
    L3 = 102.38;
    L4 = 76.91; % <--- REPLACE THIS WITH YOUR MEASURED WRIST-TO-TIP LENGTH

    % 3. Define Home Configuration (M)
    M = [1, 0, 0, 0;
         0, 1, 0, 0;
         0, 0, 1, (L1 + L2 + L3 + L4);
         0, 0, 0, 1];

    % 4. Define Screw Axes (S)
    S1 = [0; 0; 1; 0; 0; 0];
    S2 = [0; 1; 0; -L1; 0; 0];
    S3 = [0; 1; 0; -(L1 + L2); 0; 0];
    S4 = [0; 1; 0; -(L1 + L2 + L3); 0; 0];

    % 5. Compute se(3) matrices
    se3_1 = VecTose3(S1) * th1;
    se3_2 = VecTose3(S2) * th2;
    se3_3 = VecTose3(S3) * th3;
    se3_4 = VecTose3(S4) * th4;

    % 6. Compute Final Transformation Matrix
    T_final = expm(se3_1) * expm(se3_2) * expm(se3_3) * expm(se3_4) * M;

    % 7. Extract position and rotation for the function output
    R = T_final(1:3, 1:3);
    x = T_final(1, 4);
    y = T_final(2, 4);
    z = T_final(3, 4);
end