% RBE 501 - Robot Dynamics - Spring 2022
% Homework 2, Problem 1
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/06/2022
clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20; % number of random test configurations

%% Create the manipulator
% Link length values (meters)
L1 = 0.3;
L2 = 0.3;
L3 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L1, 'alpha', pi/2, 'offset', pi/2), ...
                    Revolute('a', L2, 'd', 0, 'alpha', 0), ...
                    Revolute('a', L3, 'd', 0, 'alpha', pi/2, 'offset', -pi/2)], ...
                    'name', 'RRR Manipulator');

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        -pi/12 pi/3]; % q(3)

% Display the manipulator in the home configuration
q = zeros(1,3);
robot.teach(q);


%% Part A - Calculate the screw axes
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

%S = [0 0 1 0 0 0;0 1 0 -0.3 0 0; 0 1 0 0 0 0.3]';
S = [0 0 1 0 0 0;1 0 0 0 0.3 0;1 0 0 0 0.3 -0.3]';

%% Part B - Calculate the forward kinematics with the Product of Exponentials formula
% First, let us calculate the homogeneous transformation matrix M for the
% home configuration
R = [0 1 0;
    0 0 -1;
    -1 0 0];
p = [0 0.3 0]';

M = [R,p;0 0 0 1];

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the forward kinematics
    T = fkine(S,M,q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');


%% Part C - Calculate the Space Jacobian of the manipulator
fprintf('-------------------Differential Kinematics Test------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 

% Test the correctness of the Jacobian for 100 random sets of joint
% variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(), ...
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(), ...
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand()];
    
    % Calculate the Forward Kinematics
    T = fkine(S,M,q);
    
    % Calculate the Jacobian
    J = jacob0(S,q);
    
    if plotOn
        robot.teach(q);
        title('Differential Kinematics Test');
    end
    
    % Test the correctness of the Jacobian
    Jcoords = [-skew(T(1:3,4))*J(1:3,:)+J(4:6,:); J(1:3,:)];
    assert(all(all(abs(double(robot.jacob0(q)) - Jcoords) < 1e-10)));
end

fprintf('\nTest passed successfully.\n');

%% Part D - Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');

% Calculate the twist representing the robot's home pose
currentPose = MatrixLog6(M);
currentPose = [currentPose(3,2) currentPose(1,3) currentPose(2,1) currentPose(1:3,4)']';

% Set the current joint variables
currentQ = zeros(1,3);

if plotOn
    robot.teach(currentQ);
    h = triad('matrix', M, 'tag', 'Target Pose', 'linewidth', 2.5, 'scale', 0.5);
end
     
% Generate the test configurations
q = [linspace(0,pi/2,nTests);
     linspace(0,pi/6,nTests);
     linspace(0,pi/6,nTests)];

for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate the robot's pose
    T = fkine(S,M,q(:,ii)');
    targetPose = MatrixLog6(T);
    targetPose = [targetPose(3,2) targetPose(1,3) targetPose(2,1) targetPose(1:3,4)']';
    
    if plotOn
        set(h, 'matrix', T);
        title('Inverse Kinematics Test');
        drawnow;
    end
    
    % Inverse Kinematics
    while norm(targetPose - currentPose) > 1e-3
        J = jacob0(S,q);
        
        % calculating deltaQ using Damped Least Square Method
        lambda = 0.5;
        deltaQ = J' * pinv(J*J' + lambda^2 * eye(6)) * (targetPose - currentPose);

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
end

fprintf('\nTest passed successfully.\n');

