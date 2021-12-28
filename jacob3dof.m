%
% jacob3dof.m
% Sept. 6, 2007.
% Ryuma Niiyama, ISI Lab. The University of Tokyo.
%
%

function [J1 J1w J2 J2w J3 J3w] = jacob3dof(L, theta)

S1 = sin(theta(1));
S12 = sin(theta(1) + theta(2));
S123 = sin(theta(1) + theta(2) + theta(3));
C1 = cos(theta(1));
C12 = cos(theta(1) + theta(2));
C123 = cos(theta(1) + theta(2) + theta(3));

% d[x y]' = J1 * theta(1)
J1w = [
    -L(1)*S1;
    L(1)*C1;
    1;
    ];

% d[x y]' = J1 * theta(1)
J1 = [
    -L(1)*S1;
    L(1)*C1;
    ];

% d[x y]' = J2 * d[theta(1) theta(2)]'
J2w = [
    -L(1)*S1-L(2)*S12, -L(2)*S12;
    L(1)*C1+L(2)*C12,  L(2)*C12;
    1 1;
    ];

% d[x y]' = J2 * d[theta(1) theta(2)]'
J2 = [
    -L(1)*S1-L(2)*S12, -L(2)*S12;
    L(1)*C1+L(2)*C12,  L(2)*C12];

% d[x y w]' = J3 * d[theta(1) theta(2) theta(3)]' �����N�G���h���x?{�p���x
J3w = [
    -L(1)*S1-L(2)*S12-L(3)*S123, -L(2)*S12-L(3)*S123, -L(3)*S123;
    L(1)*C1+L(2)*C12+L(3)*C123,  L(2)*C12+L(3)*C123,  L(3)*C123;
    1 1 1;
    ];
% d[x y]' = J3 * d[theta(1) theta(2) theta(3)]' �����N�G���h���x
J3 = [
    -L(1)*S1-L(2)*S12-L(3)*S123, -L(2)*S12-L(3)*S123, -L(3)*S123;
    L(1)*C1+L(2)*C12+L(3)*C123,  L(2)*C12+L(3)*C123,  L(3)*C123;
    ];

