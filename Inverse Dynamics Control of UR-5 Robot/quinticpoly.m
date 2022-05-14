function A = quinticpoly(t0,tf,q0,qf,qd0,qdf,qdd0,qddf)
    M = [1 t0 t0^2 t0^3   t0^4    t0^5;
         0 1  2*t0 3*t0^2 4*t0^3  5*t0^4;
         0 0  2    6*t0   12*t0^2 20*t0^3;
         1 tf tf^2 tf^3   tf^4    tf^5;
         0 1  2*tf 3*tf^2 4*tf^3  5*tf^4;
         0 0  2    6*tf   12*tf^2 20*tf^3];
    
    A = pinv(M) * [q0 qd0 qdd0 qf qdf qddf]';
end