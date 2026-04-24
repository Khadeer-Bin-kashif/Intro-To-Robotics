function thetas = findJointAngles(x, y, z, phi)
    % findJointAngles: Calculates Inverse Kinematics to match the PoE model.
    % 
    % ARGUMENTS:
    %   x, y, z : Target position in millimeters.
    %   phi     : Target total pitch angle in radians (from vertical Z-axis).

    % Define the precise link lengths from your PoE Forward Kinematics
    L1 = 121.8;
    L2 = 102.38;
    L3 = 102.38;
    L4 = 76.91;
    
    % Initialize an empty array to store the valid N x 4 solutions
    thetas = [];
    
    % --- STEP 1: Calculate theta1 (Base Pan) ---
    % Check for base singularity (target is perfectly on the Z-axis)
    if norm([x, y]) < 1e-6
        % Base rotation is arbitrary at the origin; choosing 0
        theta1_sols = 0; 
    else
        % Two possible base rotations: pointing towards target or away from it
        theta1_sols = [atan2(y, x), atan2(-y, -x)];
    end
    
    % --- STEP 2: Isolate the Wrist Position ---
    % In PoE, the arm starts straight up (vertical). 
    % Therefore, Z relies on cosine, and the horizontal reach (R) relies on sine.
    z_bar = z - L1 - L4 * cos(phi);
    
    % Loop through each possible base rotation
    for i = 1:length(theta1_sols)
        th1 = theta1_sols(i);
        
        % Project x and y onto the current base frame orientation
        r_xy = x * cos(th1) + y * sin(th1);
        r_bar = r_xy - L4 * sin(phi);
        
        % --- STEP 3: Solve for theta3 (Elbow) using Law of Cosines ---
        % Denominator is 2 * L2 * L3
        denominator = 2 * L2 * L3;
        D = (r_bar^2 + z_bar^2 - L2^2 - L3^2) / denominator;
        
        % FIX: Add tolerance for floating-point errors when arm is fully extended
        if abs(D) > 1 && abs(D) <= 1.001 
            D = sign(D) * 1; % Force D to be exactly 1 or -1
        end
        
        % Workspace bounds check: Skip if target is out of physical reach
        if abs(D) <= 1
            % Two possible elbow configurations: Up and Down
            th3_up = atan2(sqrt(1 - D^2), D);
            th3_down = atan2(-sqrt(1 - D^2), D);
            
            theta3_sols = [th3_up, th3_down];
            
            % --- STEP 4 & 5: Solve for theta2 and theta4 ---
            for j = 1:2
                th3 = theta3_sols(j);
                
                % Algebraic identities for the 2-link system
                k1 = L2 + L3 * cos(th3);
                k2 = L3 * sin(th3);
                
                % Calculate theta2 (Shoulder)
                % Because home is vertical, the atan2 formula shifts slightly
                th2 = atan2((k1 * r_bar - k2 * z_bar), (k1 * z_bar + k2 * r_bar));
                
                % Calculate theta4 (Wrist) from the PoE end-effector constraint
                % In PoE: phi = th2 + th3 + th4
                th4 = phi - th2 - th3;
                
                % Append this valid 4-DOF configuration as a new row
                thetas = [thetas; th1, th2, th3, th4];
            end
        end
    end
end