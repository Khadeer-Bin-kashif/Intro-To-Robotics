function DHJointAngles = servo2dh(jointAngles)
% servo2dh: Converts the servo zero coordinates for the Phantom X Pincher to DH convention zero coordinates after applying the difference in rotation offsets in between the two.
% 
% Arguments:
%   jointAngles: A 1x4 or 4x1 vector [th1, th2, th3, th4] in RADIANS.
% 
% Returns:
%   DH zero config based angles.
%   x, y, z: The Cartesian position of the end-effector in mm.
%   R: The 3x3 Rotation Matrix representing the end-effector orientation.

    % 1. Extracting individual joint angles
    th1 = jointAngles(1);
    th2 = jointAngles(2);
    th3 = jointAngles(3);
    th4 = jointAngles(4);

    % The mapping being used
    % DH = (Servo * Direction) + Shift
    DHJointAngles = zeros(1, 4);
    DHJointAngles(1) = (-1 * th1) - pi/2;
    DHJointAngles(2) = (-1 * th2) - pi/2;
    DHJointAngles(3) = (1 * th3) + 0;
    DHJointAngles(4) = (1 * th4) + 0;
end