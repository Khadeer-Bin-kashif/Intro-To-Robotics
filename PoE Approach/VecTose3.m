function se3mat = VecTose3(S)
    % Converts a 6x1 spatial velocity vector into a 4x4 se(3) matrix
    omega = S(1:3);
    v = S(4:6);
    
    omega_skew = [   0,      -omega(3),  omega(2);
                  omega(3),     0,      -omega(1);
                 -omega(2),  omega(1),     0     ];
             
    se3mat = [omega_skew, v; 
              0, 0, 0, 0];
end