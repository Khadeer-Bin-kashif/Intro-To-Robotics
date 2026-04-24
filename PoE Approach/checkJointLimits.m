%% Helper Function: Check Joint Limits
function isWithinLimits = checkJointLimits(q)
    % Limits in radians: 150 degrees = 150 * pi / 180
    limit_rad =deg2rad(150);
    
    % Check if all joints are within [-limit, limit]
    % Note: If your IK derivation used different zero-offsets than your 
    % servos, add/subtract the offsets here before checking.
    isWithinLimits = all(q >= -limit_rad) && all(q <= limit_rad);
end