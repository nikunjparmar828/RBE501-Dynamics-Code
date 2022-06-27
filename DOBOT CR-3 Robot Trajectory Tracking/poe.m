% RBE 501 - Robot Dynamics - Sping 2022
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***Nikunj Parmar***
clear, clc, close all
addpath('utils');
plotOn = false;
nTests = 20; 
%% *** ENTER THE LAST DIGIT OF YOUR WPI ID BELOW: ***
digit = 2;

%% Create the manipulator
robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;
robot.teach(zeros(1,6))


%% Calculate the forward kinematics using the Product of Exponentials
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints
S = [0 0 1 0 0 0;
     1 0 0 0 0.1348 0;
    1 0 0 0 0.4086 0;
    1 0 0 0 0.6386 0;
    0 0 1 0 -0.1283 0;
    1 0 0 0 0.7546 0]';

% Let us also calculate the homogeneous transformation matrix M for the
% home configuration

R = [0 0 1;
    -1 0 0;
    0 -1 0];
p = [0.2333 0 0.7546]';

M = [R,p;0 0 0 1];

%%---------------------------------------------------------------------------

fprintf('---------------------Forward Kinematics Test---------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%'); 
 
% Test the forward kinematics for 100 random sets of joint variables
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
    
    % Generate a random configuration
    q = [qlim(1,1) + (qlim(1,2) - qlim(1,1)) * rand(),... 
         qlim(2,1) + (qlim(2,2) - qlim(2,1)) * rand(),... 
         qlim(3,1) + (qlim(3,2) - qlim(3,1)) * rand(),... 
         qlim(4,1) + (qlim(4,2) - qlim(4,1)) * rand(),... 
         qlim(5,1) + (qlim(5,2) - qlim(5,1)) * rand(),... 
         qlim(6,1) + (qlim(6,2) - qlim(6,1)) * rand()];
    
    % Calculate the forward kinematics
    T = fkine(S,M,q);
    
    if plotOn
        robot.teach(q);
        title('Forward Kinematics Test');
    end
    
    assert(all(all(abs(double(robot.fkine(q)) - T) < 1e-10)));
end
 
fprintf('\nTest passed successfully.\n');
