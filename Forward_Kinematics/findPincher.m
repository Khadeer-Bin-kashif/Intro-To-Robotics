function [x, y, z, R] = findPincher(COMport, total_servos)

    COMport = 'COM' + string(COMport);

    arb = Arbotix('port', COMport, 'nservos', total_servos);

    servo_angles = zeros(1, 4);

    for i = 1:4
        servo_angles(i) = arb.getpos(i);
    end

    dh_angles = servo2dh(servo_angles);

    [x, y, z, R] = pincherFK(dh_angles);
end
