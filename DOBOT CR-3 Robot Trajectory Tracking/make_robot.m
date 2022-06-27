function robot = make_robot(digit)
%MAKE_ROBOT Creates the kinematic structure of the robot used in the exam
%   This is a factory method that creates the robot needed for the exam.
%
%   Inputs: digit [int] - may be used to tweak the robot model
%   Output: robot - the robot structure, created using Peter Corke's
%   robotics toolbox
%
%   Author: L. Fichera <lfichera@wpi.edu>
%   Last modified: 02/14/2022

deg = pi/180;
mm = 1e-3;

%% Create the manipulator
if mod(digit, 2) == 0
L1 = Revolute('d', 0.1348, 'a',  0.0000, 'alpha',  pi/2, 'offset', pi/2);
L2 = Revolute('d', 0.0000, 'a',  0.2738, 'alpha',  0   , 'offset', pi/2);
L3 = Revolute('d', 0.0000, 'a',  0.2300, 'alpha',  0   , 'qlim', [-150 150]*deg);
L4 = Revolute('d', 0.1283, 'a',  0.0000, 'alpha',  pi/2, 'offset', pi/2);
L5 = Revolute('d', 0.1160, 'a',  0.0000, 'alpha', -pi/2);
L6 = Revolute('d', 0.1050, 'a',  0.0000, 'alpha',  0   );

% If the digit is an odd number, change the direction of some of the axes
else
    L1 = Revolute('d', 0.1348, 'a',  0.0000, 'alpha',  pi/2, 'offset', pi/2);
    L2 = Revolute('d', 0.0000, 'a',  0.2738, 'alpha',  pi  , 'offset', pi/2);
    L3 = Revolute('d', 0.0000, 'a',  0.2300, 'alpha',  pi  , 'qlim', [-150 150]*deg);
    L4 = Revolute('d', 0.1283, 'a',  0.0000, 'alpha', -pi/2, 'offset', pi/2);
    L5 = Revolute('d',-0.1160, 'a',  0.0000, 'alpha',  pi/2);
    L6 = Revolute('d', 0.1050, 'a',  0.0000, 'alpha',  0   );
end

%% Create SerialLink object
robot = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'CR3', 'manufacturer', 'DOBOT','tool', transl([0 0 0]));
%cr3.tool = eye(4); % take the tool off the end effector

end

