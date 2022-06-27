function T = fkine(S,M,q)
    % your code here
    
    tt = eye(4);
    [row_s col_s] = size(S);
   
    for i = 1: col_s
        tt = tt * twist2ht(S(:,i),q(i));
    end

    T = tt*M;
    
end