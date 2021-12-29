%
%   Reference data for Musculoskeletal robot with ADMA
%
%   References:
%   [1]  @article{nishikawa2014musculoskeletal,
%        title={A musculoskeletal bipedal robot designed with angle-dependent moment arm for dynamic motion from multiple states},
%        author={Nishikawa, Satoshi and Tanaka, Kazutoshi and Shida, Kazuya and Fukushima, Toshihiko and Niiyama, Ryuma and Kuniyoshi, Yasuo},
%        journal={Advanced Robotics},
%        volume={28},
%        number={7},
%        pages={487--496},
%        year={2014},
%        publisher={Taylor \& Francis}
%      }

% Make all variables global
global ADMA_par_2014 mus_par_2014;

%     -- McKibben parameter --
%  [ D0     theta0      Lmax    Num]
mus_par_2014 = [
    0.008   pi*16/180   0.280   6; % GMAX
    0.008   pi*16/180   0.260   2; % IL
    0.008   pi*16/180   0.180   2; % HAM
    0.008   pi*16/180   0.200   1; % RF
    zeros(1,4); % BF
    0.008   pi*16/180   0.180   2; % VAS
    0.008   pi*16/180   0.180   1; % GAS
    zeros(1,4); % None
    0.008   pi*16/180   0.180   1; % SOL
    0.008   pi*16/180   0.180   1; % TA
];

%      -- moment arm parameter --
%  [ a        b       c       d       r          theta_Lmax]
ADMA_par_GMAX = [
     0.0179   0.0552  0.0346  0.0200  0.0110     pi/2; % GMAX hip
     zeros(1,6);
     zeros(1,6);
]';
ADMA_par_IL = [
    -0.0437  0.0438  0.0298   0.0200  0.0110     pi/6; % IL hip
     zeros(1,6);
     zeros(1,6);
]';
ADMA_par_HAM = [
     0.0400 -0.0332  0.2600  -0.0280  0.0000     pi/2; % HAM hip
     0.0000  0.0000  0.0295   0.0048  0.0280    -pi/6; % HAM knee
     zeros(1,6);
]';
ADMA_par_RF = [
    0.0000   0.0000  0.0250   0.0084  0.0250     0; % RF hip
   -0.0098   0.0350  0.0310   0.0280  0.0040     pi/2; % RF knee
    zeros(1,6);
]';
ADMA_par_BF = [
    zeros(1,6);
    zeros(1,6);
    zeros(1,6);
]';
ADMA_par_VAS = [
    zeros(1,6);
   -0.0098   0.0350  0.0310   0.0280  0.0040     2*pi/3; % VAS knee
    zeros(1,6);
]';
ADMA_par_GAS = [
    zeros(1,6);
    0.0000   0.0000  0.0250   0.0000  0.0250     0; % GAS knee
    0.0360   0.0220  0.2800   0.0250  0.0110     pi/2; % GAS ankle
]';
ADMA_par_NONE = [
    zeros(1,6);
    zeros(1,6);
    zeros(1,6);
]';
ADMA_par_SOL = [
    zeros(1,6);
    zeros(1,6);
    0.0360   0.0220  0.2500   0.0057  0.0110     5*pi/6; % SOL ankle
]';
ADMA_par_TA = [
    zeros(1,6);
    zeros(1,6);
   -0.0325   0.0100  0.2200  -0.0273  0.0000    -pi/4; % TA ankle
]';

% Create amultidimensional array
ADMA_par_2014(:,:,1) = ADMA_par_GMAX;
ADMA_par_2014(:,:,2) = ADMA_par_IL;
ADMA_par_2014(:,:,3) = ADMA_par_HAM;
ADMA_par_2014(:,:,4) = ADMA_par_RF;
ADMA_par_2014(:,:,5) = ADMA_par_BF;
ADMA_par_2014(:,:,6) = ADMA_par_VAS;
ADMA_par_2014(:,:,7) = ADMA_par_GAS;
ADMA_par_2014(:,:,8) = ADMA_par_NONE;
ADMA_par_2014(:,:,9) = ADMA_par_SOL;
ADMA_par_2014(:,:,10) = ADMA_par_SOL;