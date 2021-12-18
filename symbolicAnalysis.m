
% Initialization
close all
clear all
clear global

pkg load symbolic

% Define the number of joints
joint_num = 2

for i = 1:joint_num
    alpha(:,i) = [1 0].';
end

% Create symbolic variables for the joint angles
for i = 1:joint_num
    theta(i) = sym(strcat('theta', num2str(i)));
end

% Create symbolic variables for the link lengths
for i = 1:joint_num
    Link(i) = sym(strcat('L', num2str(i)));
end

X = forwardKinematics(Link,theta,debug=false);
celldisp(X)
