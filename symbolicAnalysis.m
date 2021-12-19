% This script executes a symbolic analysis for the Forward Kinematics function.
% Please define the number of joint to analyze the Forward Kinematics.

pkg load symbolic

% Define the number of joints here
joint_num = 2

% Create symbolic variables for the joint angles
for i = 1:joint_num
    theta(i) = sym(strcat('theta', num2str(i)));
end

% Create symbolic variables for the link lengths
for i = 1:joint_num
    Link(i) = sym(strcat('L', num2str(i)));
end

[X J] = forwardKinematics(Link,theta,debug=false);
celldisp(X)
celldisp(J)
