function T = fkine(S,M,q,frame)
    % Your code here
    tt = eye(4);
    [row_s col_s] = size(S);
    
    for i = 1: col_s
        tt = tt * twist2ht(S(:,i),q(i));
    end
    
    if strcmp(frame,'space')==1    
        T = tt*M;
    elseif strcmp(frame, 'body')==1
        T = M*tt;
    end
end