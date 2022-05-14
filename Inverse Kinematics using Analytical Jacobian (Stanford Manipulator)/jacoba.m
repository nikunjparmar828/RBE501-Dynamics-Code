function J_a = jacoba(S,M,q)    
    % your code here
    J_s = jacob0(S,q);
    J_omega_s = J_s(1:3,:);
    J_v_s = J_s(4:6,:);
    
    T = fkine(S,M,q,'space');
    P = T(1:3,4);
    
    J_a = J_v_s - skew(P)*J_omega_s;
end