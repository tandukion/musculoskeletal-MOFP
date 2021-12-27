%
%   Main Program for Maximum Output Force Profile (MOFP)
%
%   Nov 21, 2017
%   Dwindra Sulistyoutomo, ISI Lab  The University of Tokyo
%
%   References:
%   [1] @inproceedings{niiyama2010athlete,
%         title={Athlete robot with applied human muscle activation patterns for bipedal running},
%         author={Niiyama, Ryuma and Nishikawa, Satoshi and Kuniyoshi, Yasuo},
%         booktitle={2010 10th IEEE-RAS International Conference on Humanoid Robots},
%         pages={498--503},
%         year={2010},
%         organization={IEEE}
%       }
%   [2]  @article{nishikawa2014musculoskeletal,
%        title={A musculoskeletal bipedal robot designed with angle-dependent moment arm for dynamic motion from multiple states},
%        author={Nishikawa, Satoshi and Tanaka, Kazutoshi and Shida, Kazuya and Fukushima, Toshihiko and Niiyama, Ryuma and Kuniyoshi, Yasuo},
%        journal={Advanced Robotics},
%        volume={28},
%        number={7},
%        pages={487--496},
%        year={2014},
%        publisher={Taylor \& Francis}
%      }

% Initialization
close all
clear all
clear global

pkg load symbolic


%% ============== LINK PARAMETERS =========================
% Define the robot link lengths here

% --- 2010ver Athlete Robot [1] ---
L = [0.300, 0.300, 0.150]; % [L_thigh L_shank L_foot]

% --- 2014ver Jumping Athlete Robot [2] ---
% L = [0.300, 0.280, 0.15]; % [L_thigh L_shank L_foot+L_toe]

%% ================ JOINT ANGLES ===========================
% Define the joint angles in [rad]

% --- 2010 ver ---
theta = [-pi/3 -pi/2 2*pi/3]'; % [hip knee ankle] 

%% ================= MUSCLE PARAMETERS ========================
% Define the muscles working on the robot

% Define the label for each muscle here
mus_name = [
    'GMAX';
    ' IL ';
    'HAM ';
    ' RF ';
    ' BF ';
    'VAS ';
    'GAS ';
    'NONE';
    'SOL ';
    ' TA ';
    ];

%% ------------------ Force profile ------------------
% The Jacobian matrix G represents the differential relationship between the joint motion and the displacemet of the actuator output.
% Each element of G represents transmission ratio depends on the moment arm of the joint for muscle.

% --- (Simplified) Human mucles on lower extremity ---
% e:extensor, f:flexor
% GMAX : Gluteus Maximus                    -> e(hip) 
% IL   : Iliopsoas (Iliacus & Psoas Major)  -> f(hip)
% HAM  : Hamstrings                         -> e(hip), f(knee)
% RF   : Rectus Femoris                     -> f(hip), e(knee)
% BF   : short head of Biceps Femoris       ->         f(knee)
% VAS  : Vastus                             ->         e(knee)
% GAS  : Gastrocnemius                      ->         f(knee), e(ankle)
% NONE : muscle not exist in human          ->         e(knee), f(ankle)
% SOL  : Soleus                             ->                  e(ankle)
% TA   : Tibialis Anterior                  ->                  f(ankle)

% From the muscle roles defined (extensor of flexor) above, 
% We can define a base matrix for Jacobian matrix G for the moment arms of the joints
G_base = [
% [ GMAX IL HAM RF  BF  VAS GAS NON SOL TA] % muscles
    -1   1  -1   1   0   0   0   0   0   0; % hip joint
     0   0  -1   1  -1   1  -1   1   0   0; % knee joint
     0   0   0   0   0   0  -1   1  -1   1; % ankle joint
    ]';

%% ------------------  MOMENT ARM ------------------
% Define the value of moment arms of the muscles on each joint.

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
% ---------------------------------------------

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

%% ------------------  MUSCLE STIFFNESS ------------------
% Simplified muscle stiffness
%  [GMAX, IL,  HAM, RF, BF,  VAS, GAS, NON, SOL, TA]
k_uniform = [
    1     1    1    1   1    1    1    1    1    1]'; % muscles set upper boundary]

%% =================  MOFP calculation =================

% --> Define the moment arms and forces for the muscle here
% --- Using data from references ---
% Moment arms
G = G_base .* MA_uniform;
% Forces
f = f_uniform;

% Compute Forward Kinematics for end-effector position X and Jacobian Matix J
[X J] = forwardKinematics(L, theta);

% Output force matrix needs to be in a square diagonal matrix.
F = eye(size(f,1)) .* repmat(f, [1 length(f)]);

% joint torque tau = G' * F
tau = G' * F;

% joint torque tau = J' * Q
% Get Q for each joint, using left division \, so Q = J'\tau
for i = 1:length(J)
    Q(:,:,i) = J{i}' \ tau(1:size(J{i},2),:);
end

%% =================  Compliance calculation =================

k = k_uniform;

% Create Muscle compliance matrix needs to be in a square diagonal matrix.
K = eye(size(k,1)) .* repmat(k, [1 length(k)]);

Kq = G' * K * G;

for i = 1:length(J)
    C(:,:,i) = J{i} * inv( Kq(1:size(J{i},2),1:size(J{i},2)) ) * J{i}';
end

% ================= Plotting =================
drawMOFP(X,Q,C);
