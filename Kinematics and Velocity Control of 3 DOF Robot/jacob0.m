function J = jacob0(S,q)
    
    [row_q col_q] = size(q);
    
    % Matrix of a similar size equal to Jacobian    
    aa = rand(6,row_q);

    [row col] = size(S);
    
    for j = 1: col
        tt = eye(4);
        for i = 1:j
            % Calculation of Transformation Matrix
            tt = tt * twist2ht(S(:,i),q(i));
        end
        aa(:,j) = adjoint(S(:,j),tt);
        
    end
    J = aa;
end