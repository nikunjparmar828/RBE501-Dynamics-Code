% RBE 501 - Robot Dynamics - Fall 2021
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***Nikunj Parmar***
clear, clc, close all
addpath('utils');

% First, execute poe.m to load the S and M matrices
poe
close all

% Generate and display the path that the robot has to trace
t = linspace(-pi, pi, 36);
x = 0.3  * ones(1,36);
%a = 0.4;
y = (10 * (sin(t)).^3)./60;
z = (13 * cos(t) - 5 * cos(2*t) - 2 * cos(3*t) - cos(4*t))./60 + 0.3;
path = [x; y; z];

scatter3(path(1,:), path(2,:), path(3,:), 'filled');

% Set the current joint variables
currentQ = zeros(1,6);
q = zeros(1,6);
% Convert Cartesian coordinates into twists
targetPose = zeros(6,size(path,2)); % each column of this matrix is a target pose represented by a twist

for ii = 1 : size(path,2)
    % First calculate the homogeneous transformation matrix representing
    % each target pose
    R = [0 0 -1; 0 1 0; 1 0 0]';
    T = [R path(:,ii); 
         0 0 0 1];
     
    % Then perform the matrix logarithm operation to convert transformation
    % matrices into 4x4 elements of se(3)
    t = MatrixLog6(T);
    
    % Finally, "unpack" this matrix (i.e., perform the inverse of the
    % bracket operator)
    targetPose(:,ii) = [t(3,2) t(1,3) t(2,1) t(1:3,4)']';
  

end

%robot.plot(qList, 'trail', {'r', 'LineWidth', 5});

%% Calculate the inverse kinematics 
% Your code here...
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,6);

     
% Generate the test configurations
q = [linspace(0,pi/2,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/2,nTests);
     linspace(0,pi/2,nTests);
     linspace(0,pi/2,nTests);
     ];

for ii = 1 : size(path,2)
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/size(path,2)*100));
    tg = targetPose(:,ii);

    
    % Inverse Kinematics
    while norm(tg - currentPose) > 1e-3
        J = jacob0(S,currentQ);
        
        lambda = 0.1;

        deltaQ = J' * pinv(J*J' + lambda^2 * eye(6)) * (tg - currentPose);

        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        
        if plotOn
            try
                robot.teach(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end

    qList(ii,1:6) = currentQ;
end

robot.plot(qList, 'trail', {'r', 'LineWidth', 5});
fprintf('\nTest passed successfully.\n');