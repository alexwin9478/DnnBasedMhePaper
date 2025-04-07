import casadi.*

%% load trained ANN
load([pwd '\02_Parameter\LSTM_8HU_10000ep_grad_v1_mhe.mat']); % PRANAV, still load for parameters
load([pwd '\02_parameter\MHE_pmsm_net_2024_0008_0014.mat']); % ALEX
net_LSTM_mhe = MHE_pmsm_net; % only necessary with ALEX new network
% numHiddenUnits_mhe = 8; % only necessary with ALEX new network

% Layer LSTM
InputWeights_lstm_mhe = double(net_LSTM_mhe.Layers(2, 1).InputWeights);
RecurrentWeights_lstm_mhe = double(net_LSTM_mhe.Layers(2, 1).RecurrentWeights);
Bias_lstm_mhe = double(net_LSTM_mhe.Layers(2, 1).Bias);
% Layer FC
Weights_fc2_mhe = double(net_LSTM_mhe.Layers(3, 1).Weights);
Bias_fc2_mhe = double(net_LSTM_mhe.Layers(3, 1).Bias);

%% Constraints
% State variables
T_w_max = 155; T_w_min = 0; % [K]
T_r_max = 155; T_r_min = 0;
v_max = 37; v_min = 0; % [m/s], defined by transmission

% Manipulated variables load torque and rev LUT in future
M_EM_acc_max = 160.36;
M_EM_acc_min = 0; %[Nm]
M_EM_brk_max = 0; M_EM_brk_min = -160.36; %[Nm]
M_fric_brk_max = 0; M_fric_brk_min = -torque_brake; %[Nm]
dt_M_EM_acc_min = -500; dt_M_EM_acc_max = -dt_M_EM_acc_min; %min dt accel decel IPG = 4s, from excel [Nm/s]

P_EM_min = M_gen_max  * pi/30  .* n_gen_max;
P_EM_min = min(P_EM_min); % Simplification!
P_EM_max = M_mot_max  * pi/30  .* n_mot_max ;
P_EM_max = max(P_EM_max); % Simplification!

%% Deadtime, Options Solver and MPC Constants

options_mhe = struct;
options_mhe.n_states            = 2 + 2 * numHiddenUnits_mhe;
options_mhe.n_states_bound      = options_mhe.n_states - 2 * numHiddenUnits_mhe;
options_mhe.n_outputs           = 4;
options_mhe.n_controls          = 2; 
options_mhe.n_parameter         = 5; 
options_mhe.n_slacks            = 4;
options_mhe.n_gzus              = 3; % zus G Constraints U2*U1, U3*U1
options_mhe.n_eqg               = 3; % number of additional equality constraints in g (U1 * U2 = 0)
options_mhe.M                   = 40; % number of control intervals
options_mhe.P                   = 15; %40; % number of prediction intervals, P >= N
options_mhe.Ts                  = 0.10; % Time step lenght [s]
options_mhe.T                   = options_mhe.P * options_mhe.Ts; % Final time horizon length in seconds [s]
options_mhe.maxIter             = 30;  % Maximum iterations for solver (nlp steps)
options_mhe.sqp_steps           = 100; %5; % spq steps
options_mhe.nRK                 = 4; 

deadtime_T = 0;                            % Deadtime on S-Func for EM Temperatures (delay)% s
delta_k_deadtime = round(deadtime_T / options_mhe.Ts); % -

T_w_0 = 60; % initial guess for winding temperature % [°C]
T_r_0 = 60; % initial guess for winding temperature % [°C]

ht0 = [zeros(numHiddenUnits_mhe,1)];
ct0 = [zeros(numHiddenUnits_mhe,1)];
% mv_0 = [M_EM_acc_0, M_EM_brk_0, M_fric_brk_0];
options_mhe.x0 = [T_w_0; T_r_0; ht0; ct0];  % Todo Initial condition of system

