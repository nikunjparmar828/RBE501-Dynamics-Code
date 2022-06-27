function massM = MassMatrixCalculator(CurrentQ,S,M,G)
    [row, col] = size(CurrentQ);
    
    % converting matrix M with respect to the {0} frame 
    new_m = zeros(4,4,(row+1));
    new_m(:,:,1) = M(:,:,1);

    n = size(M,3);
    for i=2:n
        new_m(:,:,i) = new_m(:,:,(i-1))*M(:,:,i);
    end
    
    massM = zeros(row,row);
    
    J_b_0 = zeros(6,6,row);      
    J_b = jacobe(S,new_m, CurrentQ, row, J_b_0);
    

    for i=1:row
        massM = massM + J_b(:,:,i)' * G(:,:,i) * J_b(:,:,i);
    end
end

% % function defination 
% 
% function S = skew(v)
% %SKEW Returns the skew-symmetric matrix associated with the 3D vector
% %passed as input
% 
% if length(v) == 3
%     S = [  0   -v(3)  v(2)
%         v(3)  0    -v(1)
%         -v(2) v(1)   0];
% else
%     error('argument must be a 3-vector');
% end
% end


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



function R = axisangle2rot(omega,theta)
    a = omega(1); 
    b = omega(2); 
    c = omega(3);

    skwsm = [0 -c b;c 0 -a;-b a 0];
    R = eye(3) + sin(theta)*skwsm + (1-cos(theta))*(skwsm^2);
end


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


function J = jacob0(S,q)
% your code here
    [row_q col_q] = size(q);
    J_s = rand(6,col_q);

    [row col] = size(S);
    
    for j = 1: col
        tt = eye(4);
        for i = 1:j
            tt = tt * twist2ht(S(:,i),q(i));
        end
        J_s(:,j) = adjoint(S(:,j),tt);
        
    end

    J = J_s;

end

function J_b = jacobe(S,M,q,row,J_b)    
    % your code here
    for j = 1: row
        J_s = jacob0(S(:,1:j),q(1:j)');
        T = fkine(S(:,1:j),M(:,:,j),q(1:j)','space');
        
        a = adjoint(J_s,inv(T));
        b = zeros(6,(row-j));
        
        J_b(:,:,j) = [a b];
    end

end


function T = twist2ht(S,theta)
     %your code here     
     omega = [S(1) S(2) S(3)]; % angular vecolity     
     v = [S(4) S(5) S(6)]; % linear velocity
     
     skwm = [0 -S(3) S(2);S(3) 0 -S(1);-S(2) S(1) 0];
     
     % If needed, you can calculate a rotation matrix with:     
     R = axisangle2rot(omega,theta);         
     
     T = [R , (eye(3)*theta + (1-cos(theta))* skwm + (theta - sin(theta))*(skwm^2)) * v' ;0 0 0 1]; 
end