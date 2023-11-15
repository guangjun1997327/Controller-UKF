% clear variables
% close all

%% Calculations for the sensorfusion equations
%
%  Author: Fabian Zeil
%
%

%% Vector and equations (version 1 prediction (with integration of yaw-acceleration))

% State variables
syms a_x_c a_y_c v_x_c v_y_c psi_c r_c r_d_c b_x_a1 b_y_a1 b_x_a2 b_y_a2 ...
     b_r1 b_r2
syms x w x_d

% State vector
x = [a_x_c;     % (1)   Acceleration in x-direction of the car (CG)
     a_y_c;     % (2)        -"-        y-direction  
     v_x_c;     % (3)   Velocity in x-direction
     v_y_c;     % (4)        -"-    y-direction
     psi_c;     % (5)   Yaw angle of the car
     r_c;       % (6)   Yaw rate     -"-
     r_d_c;     % (7)   Yaw acceleration
     b_x_a1;    % (8)   Bias 1. acceleration sensor in x-direction
     b_y_a1;    % (9)                -"-               y-direction
     b_x_a2;    % (10)  Bias 2. acceleration sensor in x-direction
     b_y_a2;    % (11)               -"-               y-direction
     b_r1;      % (12)  Bias 1. gyroscope around z-axis
     b_r2];     % (13)  Bias 2.          -"-


% Noise variables
syms w_x_ac w_y_ac w_x_vc w_y_vc w_psi_c w_r_c w_r_d_c w_x_ba1 ...
     w_y_ba1 w_x_ba2 w_y_ba2 w_br1 w_br2 

% Noise vector
w = [w_x_ac;
     w_y_ac;
     w_x_vc;
     w_y_vc;
     w_psi_c;
     w_r_c;
     w_r_d_c
     w_x_ba1;
     w_y_ba1;
     w_x_ba2;
     w_y_ba2;
     w_br1;
     w_br2;
     ];
 
% System covariance matrix
Q = diag(sym('q%d',[length(w) 1]));

% System equations
x_d = [w_x_ac + w_x_ac;  % (1)
       w_y_ac + w_y_ac;  % (2)
       a_x_c + w_x_vc;   % (3)
       a_y_c + w_y_vc;   % (4)
       r_c + w_psi_c;     % (5)
       r_d_c + w_r_c;   % (6)
       w_r_d_c; % (7)
       w_x_ba1;  % (8)
       w_y_ba1;  % (9)
       w_x_ba2;  % (10)
       w_y_ba2;  % (11)
       w_br1;   % (12)
       w_br2];  % (13)
 
%% Vector and equations (version 2 prediction (yaw-acceleration as an independent state))

% % State vector
% x = [a_x_c;     % (1)   Acceleration in x-direction of the car (CG)
%      a_y_c;     % (2)        -"-        y-direction  
%      v_x_c;     % (3)   Velocity in x-direction
%      v_y_c;     % (4)        -"-    y-direction
%      psi_c;     % (5)   Yaw angle of the car
%      r_c;       % (6)   Yaw rate     -"-
%      r_d_c;     % (7)   Yaw acceleration
%      b_x_a1;    % (8)   Bias 1. acceleration sensor in x-direction
%      b_y_a1;    % (9)                -"-               y-direction
%      b_x_a2;    % (10)  Bias 2. acceleration sensor in x-direction
%      b_y_a2;    % (11)               -"-               y-direction
%      b_r1;      % (12)  Bias 1. gyroscope around z-axis
%      b_r2];     % (13)  Bias 2.          -"-
%  
% % Noise vector
% w = [w_x_ac;
%      w_y_ac;
%      w_x_vc;
%      w_y_vc;
%      w_psi_c;
%      w_r_c;
%      w_r_d_c
%      w_x_ba1;
%      w_y_ba1;
%      w_x_ba2;
%      w_y_ba2;
%      w_br1;
%      w_br2;
%      ];
% 
% 
% % System covariance matrix
% Q = diag(sym('q%d',[length(w) 1]));
% 
% % System equations
% x_d = [w_x_ac + w_x_ac;  % (1)
%        w_y_ac + w_y_ac;  % (2)
%        a_x_c + w_x_vc;   % (3)
%        a_y_c + w_y_vc;   % (4)
%        r_c + w_psi_c;     % (5)
%        w_r_c;   % (6)
%        w_r_d_c; % (7)
%        w_x_ba1;  % (8)
%        w_y_ba1;  % (9)
%        w_x_ba2;  % (10)
%        w_y_ba2;  % (11)
%        w_br1;   % (12)
%        w_br2];  % (13)
 
%% Vector and equations (correction)

% Measurement variables
syms a_x_m1 a_y_m1 a_x_m2 a_y_m2 r_m1 r_m2 v_x_m v_y_m psi_m1 psi_m2 r_d_m ...
    v_x_FL_m v_x_FR_m v_x_RL_m v_x_RR_m
syms z h

% Constants
syms d_x_a1 d_y_a1 d_z_a1 d_x_a2 d_y_a2 d_z_a2 d_x_v d_y_v d_z_v d_y_FL_T ...
    d_y_FR_T d_y_RL_T d_y_RR_T
d_y_a1 = 0;
d_y_a2 = 0;
d_y_v = 0;

% Measurement vector
z = [a_x_m1;    % (1)   Acceleration measured in x-direction (front)
     a_y_m1;    % (2)              -"-           y-direction (front)
     a_x_m2;    % (3)              -"-           x-direction (back)
     a_y_m2;    % (4)              -"-           y-direction (back)
     r_m1;      % (5)   Yaw rate measured (front)
     r_m2;      % (6)   Yaw rate measured (back)
     v_x_m;     % (7)   Velocity measured in x-direction (Slip angle sensor)
     v_y_m;     % (8)              -"-       y-direction (Slip angle sensor)
     psi_m1;    % (9)   Yaw angle measured (front magnetic field sensor)
     psi_m2;    % (10)  Yaw angle measured (back magnetic field sensor)
     r_d_m;     % (11)  Yaw acceleration calculated by measurement of both y-accelerations
%      v_x_FL_m;  % (12)  Number of revelations of the motor (front left)
%      v_x_FR_m;  % (13)             -"-                     (front right)
%      v_x_RL_m;  % (14)             -"-                     (rear left)
%      v_x_RR_m   % (15)             -"-                     (rear right)
    ]; 

 
% Measurement equation

syms d_x_r1 d_y_r1 d_z_r1 d_x_r2 d_y_r2 d_z_r2

d_y_r1 = 0;
d_y_r2 = 0;

d_r_m1 = [d_x_r1;d_y_r1;d_z_r1];
d_r_m2 = [d_x_r2;d_y_r2;d_z_r2];


syms r_x1 r_y1 r_z1 r_x2 r_y2 r_z2

r_vec1 = [r_x1;r_y1; r_z1];
r_vec2 = [r_x2;r_y2; r_z2];


syms r_d_x1 r_d_y1 r_d_z1 r_d_x2 r_d_y2 r_d_z2

r_d_vec1 = [r_d_x1;r_d_y1; r_d_z1];
r_d_vec2 = [r_d_x2;r_d_y2; r_d_z2];


syms a_x_c a_y_c a_z_c

a_c = [a_x_c;a_y_c;a_z_c];

a_m1 = a_c - cross(r_d_vec1,d_r_m1) - cross(r_vec1,cross(r_vec1,d_r_m1));
a_m2 = a_c - cross(r_d_vec2,d_r_m2) - cross(r_vec2,cross(r_vec2,d_r_m2));

h_vec = [a_m1(1) + b_x_a1;
         a_m1(2) + b_y_a1;
         a_m2(1) + b_x_a2;
         a_m2(2) + b_y_a2;];
          
h = [a_x_c - r_d_c*d_y_a1 - r_c^2*d_x_a1 + b_x_a1;      % (1)
     a_y_c + r_d_c*d_x_a1 - r_c^2*d_y_a1 + b_y_a1;      % (2)
     a_x_c - r_d_c*d_y_a2 - r_c^2*d_x_a2 + b_x_a2;      % (3)
     a_y_c + r_d_c*d_x_a2 - r_c^2*d_y_a2 + b_y_a2;      % (4)
     r_c + b_r1;                                        % (5)
     r_c + b_r2;                                        % (6)
     v_x_c - r_c*d_y_v;                                 % (7)
     v_y_c + r_c*d_x_v;                                 % (8)
     psi_c;                                             % (9)
     psi_c;                                             % (10)
     r_d_c + b_y_a1/d_x_a1 + b_y_a2/d_x_a2;             % (11)
%      v_x_c - r_c*d_y_FL_T;                              % (12)
%      v_x_c - r_c*d_y_FR_T;                              % (13)
%      v_x_c - r_c*d_y_RL_T;                              % (14)
%      v_x_c - r_c*d_y_RR_T                               % (15)
    ];
% Measurement covariance matrix
R = diag(sym('r%d',[length(h) 1]));

%% Derivation of system matrices

syms F W dt

F = jacobian(x_d,x);
A = F*dt + eye(length(F));

W = jacobian(x_d,w);

%% Derivation of measurement matrices

syms H V S_b

H = jacobian(h,x);

V = eye(length(h));

I = eye(length(x));

% Observability matrix
S_b = [H;
       H*A;
       H*A^2;
       H*A^3;
       H*A^4;
       H*A^5;
       H*A^6;
       H*A^7;
       H*A^8;
       H*A^9;
       H*A^10;
       H*A^11;
       H*A^12];

%% Calculation of prediction and correction equations

% Vectors and matrices

% syms x_new_pred x_new_corr 
% syms P P_new_pred P_new_corr K
% 
% % pd = sym('P%d',[length(x) 1]);
% P = diag(sym('p%d',[length(x) 1]));
% 
% 
% % Prediction
% x_new_pred = x + x_d*dt;
% P_new_pred = F*P*F' + W*Q*W';

% Correction

% K = P_new_pred*H'/(H*P_new_pred*H'+V*R*V');
% x_new_corr = x + K*(z-h);
% P_new_corr = (I - K*H)*P_new_pred;
% 
% P_new_corr_bool = find(P_new_corr);
