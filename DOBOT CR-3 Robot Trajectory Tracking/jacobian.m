% RBE 501 - Robot Dynamics - Spring 2022
% Midterm Exam
% Worcester Polytechnic Institute
%
% Student: ***Nikunj Parmar***
clear, clc, close all
addpath('utils');

%% *** ENTER THE LAST DIGIT OF YOUR WPI ID BELOW: ***
digit = 2;

%% Create the manipulator
robot = make_robot(digit);
n = robot.n;
qlim = robot.qlim;

%% Calculate the Jacobian matrix in the home configuration

q = [0 0 0 0 0 0]';

S = [0 0 1 0 0 0;
     1 0 0 0 0.1348 0;
    1 0 0 0 0.4086 0;
    1 0 0 0 0.6386 0;
    0 0 1 0 -0.1283 0;
    1 0 0 0 0.7546 0]';

J = jacob0(S,q);
