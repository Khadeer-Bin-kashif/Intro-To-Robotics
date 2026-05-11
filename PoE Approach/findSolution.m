function optimalSolution = findSolution(x, y, z, phi, fullConfig)
    % Robot model for collision checking
    
    currentConfig = fullConfig(1:4);

    
    % 1. Get all possible analytical IK solutions
    all_solutions = findJointAngles(x, y, z, phi);
    
    valid_solutions = [];
    costs = [];
    
    % Weights for the objective function (penalizing base/shoulder more)
    b = [2.0, 1.5, 1.1, 1.0]; 
 
    for i = 1:size(all_solutions, 1)
        q_sol = all_solutions(i, :);
 
        % ---------------------------------------------------------
        % CRITICAL FIX: Wrap angles to [-pi, pi] BEFORE doing anything else!
        % This forces 246 degrees to become -113 degrees BEFORE the limit check.
        % ---------------------------------------------------------
        q_sol = mod(q_sol + pi, 2*pi) - pi;
        
        % Check 1: Joint Limits [-150 deg, 150 deg]
        if ~checkJointLimits(q_sol)
            continue;
        end
        
        % % Check 2: Collision-Free Path
        % if checkSelfCollision(currentConfig, q_sol)
        %     continue;
        % end
        
        % SOLUTION IS VALID - Calculate Cost
        valid_solutions = [valid_solutions; q_sol];
        
        current_wrapped = mod(currentConfig + pi, 2*pi) - pi;
        diff = q_sol - current_wrapped;
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
