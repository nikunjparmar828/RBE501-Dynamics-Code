function R = axisangle2rot(omega,theta)     
    % your code here     
    a = omega(1);      
    b = omega(2);      
    c = omega(3);

    % Asymmetric Matrix of omega vector
    skwsm = [0 -c b;c 0 -a;-b a 0];    

    % calculating Rotation matrix
    R = eye(3) + sin(theta)*skwsm + (1-cos(theta))*(skwsm^2); 
end