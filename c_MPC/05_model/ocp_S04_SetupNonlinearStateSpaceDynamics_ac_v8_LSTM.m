%% BOF
timestep = options.Ts; 

%% Hidden Layer LSTM
% Recurrent weights
R_i = RecurrentWeights_lstm(1:numHiddenUnits,:);
R_f = RecurrentWeights_lstm(numHiddenUnits+1:2*numHiddenUnits,:);
R_g = RecurrentWeights_lstm(2*numHiddenUnits+1:3*numHiddenUnits,:);
R_o = RecurrentWeights_lstm(3*numHiddenUnits+1:end,:);

% input weights
w_i = InputWeights_lstm(1:numHiddenUnits,:);
w_f = InputWeights_lstm(numHiddenUnits+1:2*numHiddenUnits,:);
w_g = InputWeights_lstm(2*numHiddenUnits+1:3*numHiddenUnits,:);
w_o = InputWeights_lstm(3*numHiddenUnits+1:end,:);

% bias weights
b_i = Bias_lstm(1:numHiddenUnits,:);
b_f = Bias_lstm(numHiddenUnits+1:2*numHiddenUnits,:);
b_g = Bias_lstm(2*numHiddenUnits+1:3*numHiddenUnits,:);
b_o = Bias_lstm(3*numHiddenUnits+1:end,:);

% ut is inputs for the LSTM: rotational speed, torque, T_w, T_r
% transformation speed and rotational speed: x_rpm = (v * FDR * 30) / (pi * r_dyn);
inputs_LSTM = [(v * FDR * 30) / (pi * r_dyn * n_em_max); (M_EM_acc + M_EM_brk) / M_EM_acc_max;...
    (T_w - T_cool) / T_range; (T_r - T_cool) / T_range]; % Normalized

% Candidate 
i_t = (1+exp(-(w_i*inputs_LSTM + R_i*ht + b_i))).^(-1); % Input gate (sigmoid function)with size HU*1
f_t = (1+exp(-(w_f*inputs_LSTM + R_f*ht + b_f))).^(-1); % Forget gate
g_t = tanh(w_g*inputs_LSTM + R_g*ht + b_g); % Cell candidate
o_t = (1+exp(-(w_o*inputs_LSTM + R_o*ht + b_o))).^(-1); % Output gate

ct_1 = f_t.*ct + i_t.*g_t;   % updating cell states 
ht_1 = o_t.*tanh(ct); % updating hidden states and output of layer

%% Hidden Layer 2 FC2
Z_fc2 = Weights_fc2*ht + Bias_fc2; % fully connected 7 Output: (dTw/dt, dTr/dt) -> 1 degree / 0.1 s

%% integration step of dTw, dTr
T_w_1 = timestep * (Z_fc2(1)' * (YMax - YMin) + YMin) + T_w; % Denormalized
T_r_1 = timestep * (Z_fc2(2)' * (YMax - YMin) + YMin) + T_r; %

%% vehicle dynmics
% discretize the explicit ODE of v_dot
% v_dot = f ( x, u, p)     -> v_dot!!!!! =! v_next
% v_dot ~ v ²
% -> discretize by integration to v_next = f (x,u,p)
disp(v)
v_dot = 1/Mv * ((FDR / r_dyn * (M_EM_acc + M_EM_brk + M_fric_brk)) - 0.5*Cd*Af*dens*v*v - fr*Mv*grav*cos(phi) - Mv*grav*sin(phi));
ode = casadi.Function('ode', {v, M_EM_acc, M_EM_brk, M_fric_brk, phi}, {v_dot}, ...
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
expr_phi = [T_w_1; T_r_1; v_1; ht_1; ct_1]; % next states / x_(k+1)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% output equation
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

y = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
% Lagrange Term, integral costs over the full horizon
y_e = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
% Mayer term, terminal costs at the end of the horizon



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% nonlinear constraints
%
% y = h(x,u)
% yN = hN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

h = [M_EM_acc * M_EM_brk; M_EM_acc * M_fric_brk; (abs(M_EM_brk)+M_EM_acc)*v_1*FDR/r_dyn; (v_r-v_1)]; 
h_e = [M_EM_acc * M_EM_brk; M_EM_acc * M_fric_brk; (abs(M_EM_brk)+M_EM_acc)*v_1*FDR/r_dyn; (v_r-v_1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% linear constraints
%
% y = g(x,u)
% yN = gN(xN)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
g = [];
g_e = [];