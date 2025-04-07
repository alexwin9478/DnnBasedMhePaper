%% BOF

timestep = options_mhe.Ts; 

%% Hidden Layer 1 LSTM

% Recurrent weights HU * HU
R_i = RecurrentWeights_lstm_mhe(1:numHiddenUnits_mhe,:);
R_f = RecurrentWeights_lstm_mhe(numHiddenUnits_mhe+1:2*numHiddenUnits_mhe,:);
R_g = RecurrentWeights_lstm_mhe(2*numHiddenUnits_mhe+1:3*numHiddenUnits_mhe,:);
R_o = RecurrentWeights_lstm_mhe(3*numHiddenUnits_mhe+1:end,:);

% input weights HU * 4
w_i = InputWeights_lstm_mhe(1:numHiddenUnits_mhe,:);
w_f = InputWeights_lstm_mhe(numHiddenUnits_mhe+1:2*numHiddenUnits_mhe,:);
w_g = InputWeights_lstm_mhe(2*numHiddenUnits_mhe+1:3*numHiddenUnits_mhe,:);
w_o = InputWeights_lstm_mhe(3*numHiddenUnits_mhe+1:end,:);

% bias weights HU * 1
b_i = Bias_lstm_mhe(1:numHiddenUnits_mhe,:);
b_f = Bias_lstm_mhe(numHiddenUnits_mhe+1:2*numHiddenUnits_mhe,:);
b_g = Bias_lstm_mhe(2*numHiddenUnits_mhe+1:3*numHiddenUnits_mhe,:);
b_o = Bias_lstm_mhe(3*numHiddenUnits_mhe+1:end,:);

% ut is inputs for the LSTM: rotational speed, torque, T_w, T_r
% transformation speed and rotational speed: x_rpm = (v * FDR * 30) / (pi * r_dyn);
inputs_LSTM_mhe = [(v * FDR * 30) / (pi * r_dyn * n_em_max); (M_EM_acc + M_EM_brk) / M_EM_acc_max;...
    (T_w - T_cool) / T_range; (T_r - T_cool) / T_range]; % Normalized
% Candidate
i_t_mhe = (1+exp(-(w_i*inputs_LSTM_mhe + R_i*ht + b_i))).^(-1); % Input gate (sigmoid function)with size HU*1
f_t_mhe = (1+exp(-(w_f*inputs_LSTM_mhe + R_f*ht + b_f))).^(-1); % Forget gate
g_t_mhe = tanh(w_g*inputs_LSTM_mhe + R_g*ht + b_g); % Cell candidate
o_t_mhe = (1+exp(-(w_o*inputs_LSTM_mhe + R_o*ht + b_o))).^(-1); % Output gate

ct_1 = f_t_mhe.*ct + i_t_mhe.*g_t_mhe;   % updating cell states 
ht_1 = o_t_mhe.*tanh(ct); % updating hidden states and output of layer
%% Hidden Layer 2 FC2

Z_fc2_mhe = Weights_fc2_mhe*ht + Bias_fc2_mhe; % fully connected 7 Output: (dTw/dt, dTr/dt) -> 1 degree / 0.1 s
% integration step of dTw, dTr
T_w_1 = timestep * (Z_fc2_mhe(1)' * (YMax_mhe - YMin_mhe) + YMin_mhe) + T_w;
T_r_1 = timestep * (Z_fc2_mhe(2)' * (YMax_mhe - YMin_mhe) + YMin_mhe) + T_r;

%% vehicle dynmics

% SECTION FOR CALCULATING THE VELOCITY AT THE NEXT TIME STEP

% discretize the explicit ODE of v_dot
% v_dot = f ( x, u, p)     -> v_dot!!!!! =! v_next
% v_dot ~ v ²
% -> discretize by integration to v_next = f (x,u,p)
disp(v)
v_dot_mhe = 1/Mv * ((FDR / r_dyn * (M_EM_acc + M_EM_brk + M_fric_brk)) - 0.5*Cd*Af*dens*v*v - fr*Mv*grav*cos(phi) - Mv*grav*sin(phi));
ode = casadi.Function('ode', {v, M_EM_acc, M_EM_brk, M_fric_brk, phi}, {v_dot_mhe}, ...
    {'v', 'M_EM_acc', 'M_EM_brk', 'M_fric_brk', 'phi'},{'v_dot'});
disp(ode)

% Set up explicit Euler first order
% k1 = ode(v, M_EM_acc, M_EM_brk, M_fric_brk, phi);
% v_1 = v + timestep * k1;

% Set up explicit runge kutta 4 order, RK4
k1 = ode(v, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k2 = ode(v + timestep/2*k1, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k3 = ode(v + timestep/2*k2, M_EM_acc, M_EM_brk, M_fric_brk, phi);
k4 = ode(v + timestep*k3, M_EM_acc, M_EM_brk, M_fric_brk, phi);
v_1 = v + timestep/6 * (k1 + 2*k2 + 2*k3 + k4);

%% Discrete Model
% expr_phi = [T_w_1; T_r_1; v_1; ht_1; ct_1]; % next states / x_(k+1) OLD EQUATION
% expr_phi = [T_w_1; T_r_1; ht_1; ct_1];
% expr_phi = expr_phi + sym_w;
expr_phi = [T_w_1 + w_Tw; T_r_1+ w_Tr; ht_1; ct_1];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output equation (not required if linear is used)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Alex
y = [T_w_1; T_r_1; w_Tw; w_Tr];
y_0  = [T_w_1; T_r_1; w_Tw; w_Tr; T_w_1 + w_Tw; T_r_1+ w_Tr]; % The last 2 T_w_1 and T_r_1 will be used for arrival cost. Check if they have an influence.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear constraints
%
% y = h(x,u)
% yN = hN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% h = []; 
% h_e = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear constraints
%
% y = g(x,u)
% yN = gN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = [];
g_e = [];
