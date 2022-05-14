function twist_inB = adjoint(twist_inA,T_AB)

    V = twist_inA;

    % Rotation Matrix
    R = [T_AB(1,1:3);T_AB(2,1:3);T_AB(3,1:3)];
    
    % Translation Vector
    P = T_AB(1:3,4);

    % Twist
    vb = [V(4) V(5) V(6)];
    
    %Assymetrix matrix for P
    skew_P = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    twist_inB = [R, [0 0 0;0 0 0;0 0 0];skew_P*R, R] * V;
    
end
