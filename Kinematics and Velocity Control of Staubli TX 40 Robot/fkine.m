function T = fkine(S,M,q)

    tt = eye(4);
    [row_s col_s] = size(S);
   
    for i = 1: col_s
        % Calculating Tranformatiob matrix 
        tt = tt * twist2ht(S(:,i),q(i));
    end

    T = tt*M;
    
end