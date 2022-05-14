function J_b = jacobe(S,M,q)    
    % your code here
    J_s = jacob0(S,q);
    T = fkine(S,M,q,'space');
    R = [T(1,1:3);T(2,1:3);T(3,1:3)];
    P = T(1:3,4);
    
    skew_P = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    J_b = inv([R, [0 0 0;0 0 0;0 0 0];skew_P*R, R])*J_s;
end
