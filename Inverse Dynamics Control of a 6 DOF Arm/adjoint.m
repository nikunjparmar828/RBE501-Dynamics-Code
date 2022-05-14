function Vb = adjoint(Va,T)
    V = Va;
    R = [T(1,1:3);T(2,1:3);T(3,1:3)];
    P = T(1:3,4);
    vb = [V(4) V(5) V(6)];
    
    
    skew_P = [0 -P(3) P(2);
              P(3) 0 -P(1);
              -P(2) P(1) 0];
    
    Vb = [R, [0 0 0;0 0 0;0 0 0];skew_P*R, R] * V;
    %skew_V = [skew_omg, [V(4) V(5) V(6)]'; 0 0 0 0];
end

