%function twist_inB = adjoint(twist_inA,T_AB)
% twist_inB = ...
%end



function twist_inB = adjoint(twist_inA,T_AB)
    % your code here
    %omega = [V(1) V(2) V(3)];
    V = twist_inA;
    R = [T_AB(1,1:3);T_AB(2,1:3);T_AB(3,1:3)];
    P = T_AB(1:3,4);
    vb = [V(4) V(5) V(6)];
    
    
    skew_P = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    twist_inB = [R, [0 0 0;0 0 0;0 0 0];skew_P*R, R] * V;
    %skew_V = [skew_omg, [V(4) V(5) V(6)]'; 0 0 0 0];
    %Vtrans = T * skew_V * pinv(T); 
end
