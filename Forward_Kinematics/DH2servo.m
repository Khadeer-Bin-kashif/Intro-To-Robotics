function servoAngles = DH2servo(DHjointAngles)

    % 1. Extracting individual joint angles
    th1 = DHjointAngles(1);
    th2 = DHjointAngles(2);
    th3 = DHjointAngles(3);
    th4 = DHjointAngles(4);

    servoAngles(1) = -1 * (th1 + pi/2);
    servoAngles(2) = -1 * (th2 + pi/2);
    servoAngles(3) = th3;
    servoAngles(4) = th4;
end
