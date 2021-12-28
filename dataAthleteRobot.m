
%% ------------------ LINK PARAMETERS ------------------
% Define the robot link lengths here

% --- 2010ver Athlete Robot [1] ---
L_athlete_robot = [0.300, 0.300, 0.150]; % [L_thigh L_shank L_foot]

%% ------------------  MOMENT ARM ------------------
%  [GMAX,   IL,     HAM,    RF,     BF,     VAS,    GAS,    NONE,   SOL,    TA   ]

% ------ PRE-DEFINED values from references ------
% --- 2010ver Athlete Robot [1] ---
% Simplified moment arm matrices
% #1 Uniform
MA_uniform = [
    1.000   1.000   1.000   1.000   0.000   0.000   0.000   0.000   0.000   0.000;
    0.000   0.000   1.000   1.000   1.000   1.000   1.000   1.000   0.000   0.000;
    0.000   0.000   0.000   0.000   0.000   0.000   1.000   1.000   1.000   1.000;
    ]';
% #2 anthropomorphic
MA_anthropomorphic = [
    2.000   2.000   0.500   0.500   0.000   0.000    0.000   0.000   0.000   0.000;
    0.000   0.000   0.500   0.500   0.000   2.000    0.500   0.000   0.000   0.000;
    0.000   0.000   0.000   0.000   0.000   0.000    0.500   0.000   2.000   0.500;
    ]';
% #3 symmetric mono-articular
MA_symmetric_mono_articular = [
    2.000   2.000   0.000   0.000   0.000   0.000   0.000   0.000   0.000   0.000;
    0.000   0.000   0.000   0.000   2.000   2.000   0.000   0.000   0.000   0.000;
    0.000   0.000   0.000   0.000   0.000   0.000   0.000   0.000   1.000   1.000;
    ]';
% #4 athlete robot 
MA_robot = [
    0.050   0.000   0.060   0.024   0.000   0.000   0.000   0.000   0.000   0.000;
    0.000   0.000   0.020   0.024   0.000   0.024   0.020   0.000   0.000   0.000;
    0.000   0.000   0.000   0.000   0.000   0.000   0.050   0.000   0.050   0.035;
    ]';


%% ------------------  MUSCLE OUTPUT FORCE (TENSION FORCE) -----------------------------

% ------ PRE-DEFINED values from references ------
% --- 2010ver Athlete Robot [1] ---
% Simplified force vectors
%  [GMAX, IL,  HAM, RF, BF,  VAS, GAS, NON, SOL, TA] (N)
f_uniform = [
    1     1    1    1   1    1    1    1    1    1]'; % muscles set upper boundary

f_robot_2010 = [
    3200  1600 1600 800 0    2400 1600 0    0    200]';

% ---------------------------------------------