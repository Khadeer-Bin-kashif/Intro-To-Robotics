function [success, th1] = positionJaw(position)
    % measured parameters of the 2R planar model assumed in the jaw
    L1 = 8.68;
    L2 = 25.91;
    
    th1 = NaN; 

    % Triangle constraints
    x_min = L2 - L1;
    x_max = L1 + L2;
    
    % Triangle constraints checking
    if position < x_min || position > x_max
        fprintf('\n[ERROR] Gripper command failed!\n');
        fprintf('Requested position (%.2f mm) is geometrically impossible.\n', position);
        fprintf('Valid range is between %.2f mm and %.2f mm.\n', x_min, x_max);
        
        success = false;
        
        return; 
    end
    
    numerator = L1^2 + position^2 - L2^2;
    denominator = 2 * L1 * position;
    
    th1 = acos(numerator / denominator);
    
    success = true; 
end