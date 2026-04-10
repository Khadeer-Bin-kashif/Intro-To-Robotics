function solutions = findJointAngles(x, y, z, phi)
    % findJointAngles Calculates the inverse kinematics for the Phantom X Pincher
    % Inputs: 
    %   x, y, z - Desired end-effector coordinates (in mm)
    %   phi     - Desired orientation angle of the gripper
    % Output:
    %   solutions - N x 4 matrix of valid joint angles [theta1, theta2, theta3, theta4]
    
    solutions = [];
    
    % Link lengths
    L1 = 103; 
    L2 = 102; 
    L3 = 71;  
    
    % Step 1: Calculate the two possible base angles (theta1)
    theta1_front = atan2(y, x);
    theta1_back = atan2(-y, -x); % The arm can reach "backwards" over itself
    
    % Iterate over both base configurations
    for is_front = [true, false]
        if is_front
            theta1 = theta1_front;
            r = sqrt(x^2 + y^2);
        else
            theta1 = theta1_back;
            r = -sqrt(x^2 + y^2); % Radial distance must be negative if pointing backwards
        end
        
        % Step 2: Find the wrist center (r_bar, z_bar)
        % As derived: r_bar = r - 71*cos(phi) and z_bar = z + 71*sin(phi)
        r_bar = r - L3 * cos(phi);
        z_bar = z + L3 * sin(phi); 
        
        % Step 3: Solve for theta3 using the law of cosines implementation
        % r_bar^2 + z_bar^2 = L1^2 + L2^2 + 2*L1*L2*cos(theta3)
        D = (r_bar^2 + z_bar^2 - L1^2 - L2^2) / (2 * L1 * L2);
        
        % Collision/Reachability check: if D is outside [-1, 1], the target is out of reach
        if D < -1 || D > 1
            continue; % Skip this configuration and move to the next
        end
        
        % Two possible elbow configurations: up and down
        theta3_up = atan2(sqrt(1 - D^2), D);
        theta3_down = atan2(-sqrt(1 - D^2), D);
        
        for theta3 = [theta3_up, theta3_down]
            
            % Step 4: Solve for theta2
            % Using your identity expansion: k1 = 103 + 102cos(theta3), k2 = 102sin(theta3)
            k1 = L1 + L2 * cos(theta3);
            k2 = L2 * sin(theta3);
            
            % Isolated result from your cross-multiplication
            theta2 = atan2(k2*r_bar + k1*z_bar, k1*r_bar - k2*z_bar);
            
            % Step 5: Solve for theta4
            % Using your orientation constraint: phi = theta3 - theta2 + theta4
            theta4 = phi - theta3 + theta2;
            
            % Append the solution as a new row in the matrix
            solutions = [solutions; theta1, theta2, theta3, theta4];
        end
    end
end
 