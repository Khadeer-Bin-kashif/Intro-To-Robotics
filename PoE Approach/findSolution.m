function optimalSolution = findSolution(x, y, z, phi, fullConfig)
    % Robot model for collision checking
    currentConfig = fullConfig(1:4);
   % persistent robot
    
 
    % 1. Get all possible analytical IK solutions
    % Assumes findJointAngles returns an N x 4 matrix
    all_solutions = findJointAngles(x, y, z, phi)
    
    valid_solutions = [];
    costs = [];
    
    % Weights for the objective function (penalizing base/shoulder more)
    b = [2.0, 1.5, 1.1, 1.0]; 
 
    for i = 1:size(all_solutions, 1)
        q_sol = all_solutions(i, :);
 
        
        % Check 1: Joint Limits [-150 deg, 150 deg]
        if ~checkJointLimits(q_sol)
            fprintf('Solution %d rejected: Joint Limits exceeded.\n', i);
            fprintf('Angles (deg): %s\n', num2str(rad2deg(q_sol)));
            continue;
        end
        
        % Check 2: Collision-Free Path
        % Uses the function we wrote previously
        if checkSelfCollision(currentConfig, q_sol)
            fprintf('Solution %d rejected: Collision detected along path.\n', i);
            continue;
        end
        
        % SOLUTION IS VALID - Calculate Cost
        valid_solutions = [valid_solutions; q_sol];
        
        % --- Shortest Path Logic per Task 6.5 ---
        % Manual Step 1: Rewrite angles in [-pi, pi] BEFORE computing difference
        q_sol_wrapped = mod(q_sol + pi, 2*pi) - pi;
        current_wrapped = mod(currentConfig + pi, 2*pi) - pi;
        
        % Manual Step 2: Compute difference
        diff = q_sol_wrapped - current_wrapped;
        
    
        shortest_diff = mod(diff + pi, 2*pi) - pi;
        
        current_cost = sum(b .* abs(shortest_diff));
        costs = [costs; current_cost];
    end
 
    % 3. Determine the optimal solution
    if isempty(valid_solutions)
        error('No realizable or collision-free IK solution found for this point.');
    else
        [~, minIdx] = min(costs);
        optimalSolution = valid_solutions(minIdx, :);
    end
end
