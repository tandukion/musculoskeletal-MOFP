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
global ADMA_par_2014;

%     -- McKibben parameter --
%  [ D0     theta0      l_mus   l_base   theta1_l_base]
mus_param = [
    0.008   pi*16/180   0.28    0.28     pi*90/180;
];

%      -- moment arm parameter --
%  [ a        b       c       d       r ]
ADMA_par_GMAX = [
     0.0179   0.0552  0.0346  0.0200  0.0110; % GMAX hip
     zeros(1,5);
     zeros(1,5);
]';
ADMA_par_IL = [
    -0.0437  0.0438  0.0298   0.0200  0.0110; % IL hip
     zeros(1,5);
     zeros(1,5);
]';
ADMA_par_HAM = [
     0.0400 -0.0332  0.2600  -0.0280  0.0000; % HAM hip
     0.0000  0.0000  0.0295   0.0048  0.0280; % HAM knee
     zeros(1,5);
]';
ADMA_par_RF = [
    0.0000   0.0000  0.0250   0.0084  0.0250; % RF hip
   -0.0098   0.0350  0.0310   0.0280  0.0040; % RF knee
    zeros(1,5);
]';
ADMA_par_BF = [
    zeros(1,5);
    zeros(1,5);
    zeros(1,5);
]';
ADMA_par_VAS = [
    zeros(1,5);
   -0.0098   0.0350  0.0310   0.0280  0.0040; % VAS knee
    zeros(1,5);
]';
ADMA_par_GAS = [
    zeros(1,5);
    0.0000   0.0000  0.0250   0.0000  0.0250; % GAS knee
    0.0360   0.0220  0.2800   0.0250  0.0110; % GAS ankle
]';
ADMA_par_NONE = [
    zeros(1,5);
    zeros(1,5);
    zeros(1,5);
]';
ADMA_par_SOL = [
    zeros(1,5);
    zeros(1,5);
    0.0360   0.0220  0.2500   0.0057  0.0110; % SOL ankle
]';
ADMA_par_TA = [
    zeros(1,5);
    zeros(1,5);
   -0.0325   0.0100  0.2200  -0.0273  0.0000; % TA ankle
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