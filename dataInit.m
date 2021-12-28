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

% Simplified muscle stiffness
%  [GMAX, IL,  HAM, RF, BF,  VAS, GAS, NON, SOL, TA]
k_uniform = [
    1     1    1    1   1    1    1    1    1    1]'; % muscles set upper boundary]

