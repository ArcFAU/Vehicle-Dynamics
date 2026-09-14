function tir = Hoosier_16x7_5_10x8_R20_82kPa()
%% Tire Parameters: Formula Student        %%
% ----------------------------------------- %
% Version: V1.0 - 2024.09.14                %
% Compatible ADTM Release: ADTM_1.4         %
% Author: Simon Frank simon.sf.frank@tum.de %
% Modified by:                              %
% Tim Braungardt tim.braungardt@voltages.de %
% Info:                                     %
% For lateral data:                         %
% Hoosier 43075 16x7.5-10 R20, 8 inch rim   %
% For longitudinal data:                    %
% Hoosier 43100 18.0x6.0-10 R20, 7 inch rim %
% TTC Round 9                               %
%                                           %
% Data extract import data:                 %
%   Fz = 575 (interpolated), 2*Fz = 1150    %
%   IA = 0, P = 82, V = 40                  %
% ----------------------------------------- %

% General
tir.r_0 = 0.205;    % Unloaded tire radius in m
tir.b_0 = 0.1905;   % Tire tread width in m
tir.c_x = 140E3;    % Longitudinal stiffness in N/m
tir.c_y = 65E3;     % Lateral stiffness in N/m
tir.c_z = 98E3;    % Vertical stiffness in N/m
tir.c_psi = 400;    % Torsional stiffness in Nm/rad
tir.d_x = 400;      % Longitudinal damping in N*s/m
tir.d_y = 300;      % Lateral damping in N*s/m
tir.d_z = 150;      % Vertical damping in N*s/m
tir.d_psi = 1;      % Torsional damping in Nm*s/rad
tir.a_r = 0.02;     % Rolling resistance coefficient in N/N
tir.dr_s= 1E-7;     % Tire radius increase with omega^2 in m/(rad/s)^2

% Tire model properties
tir.F_zN  = 575;    % Nominal normal force in N
tir.F_zMax= 4000;   % Maximum normal force in N
tir.R_N   = 0.05;   % Tire cross section roundness in [0,1]
tir.v_n   = 0.5;      % Velocity offset for slip calculation in m/s
fac_sS    = 1.67;   % Factor of s_M where full sliding starts in -
fac_FS    = 0.95;   % Ratio of sliding to peak force in -
fac_NoTz1 = 1.5;    % Factor of s_yM for first T_Align zero in -
fac_NoTz2 = 1.9;    % Factor of s_yM for second T_Align zero in -
effCamber = 0.2;    % Camber force effectiveness in -
effBore   = 0.7;    % Bore torque effectiveness in -

% F_x @ F_zN
tir.dF_x01  = 30248;

tir.s_xMA1  = 0.122;
tir.F_xMA1  = 1359;
tir.s_xSA1  = tir.s_xMA1*fac_sS;
tir.F_xSA1  = tir.F_xMA1*fac_FS;

tir.s_xMB1  = 0.152;
tir.F_xMB1  = 1432;
tir.s_xSB1  = tir.s_xMB1*fac_sS;
tir.F_xSB1  = tir.F_xMB1*fac_FS;

% F_x @ 2*F_zN
tir.dF_x02  = 56000;

tir.s_xMA2  = 0.114;
tir.F_xMA2  = 2580;
tir.s_xSA2  = tir.s_xMA2*fac_sS;
tir.F_xSA2  = tir.F_xMA2*fac_FS;

tir.s_xMB2  = 0.130;
tir.F_xMB2  = 2730;
tir.s_xSB2  = tir.s_xMB2*fac_sS;
tir.F_xSB2  = tir.F_xMB2*fac_FS;

% F_y @ F_zN
tir.dF_y01  = 23590;
tir.s_yM1   = 0.186;
tir.F_yM1   = 1492;
tir.s_yS1   = tir.s_yM1*fac_sS;
tir.F_yS1   = tir.F_yM1*fac_FS;

% F_y @ 2*F_zN
tir.dF_y02  = 36000;
tir.s_yM2   = 0.162;
tir.F_yM2   = 2710;
tir.s_yS2   = tir.s_yM2*fac_sS;
tir.F_yS2   = tir.F_yM2*fac_FS;

% Trail @ F_zN
tir.nu_01 = 0.3;
tir.s_y01 = tir.s_yM1*fac_NoTz1;
tir.s_yE1 = tir.s_yM1*fac_NoTz2;

% Trail @ 2*F_zN
tir.nu_02 = 0.35;
tir.s_y02 = tir.s_yM2*fac_NoTz1;
tir.s_yE2 = tir.s_yM2*fac_NoTz2;

% Other parameters
tir.eta_c = effCamber;
tir.eta_b = effBore;
end
