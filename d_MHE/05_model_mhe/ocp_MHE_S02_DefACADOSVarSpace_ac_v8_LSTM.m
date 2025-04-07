import casadi.*

% States
% Winding / stator temperatur (°C) 
T_w = SX.sym('T_w', 1);
% Rotor temperature (°C) 
T_r = SX.sym('T_w', 1);
% Vehicle speed ( m/s ) 
v = SX.sym('v', 1);

ht = SX.sym('ht',numHiddenUnits_mhe); % HiddenhHiddeH
ct = SX.sym('ct',numHiddenUnits_mhe); % Cell states

x = vertcat(T_w, T_r, ht, ct); % states, add all states

%% differential states
v_dot_mhe = SX.sym('v_dot', 1); % acceleration, this is discretized in ocp:S04

%% Algebraic variables
i_t_mhe = SX.sym('i_t',numHiddenUnits_mhe); % Input gate (sigmoid function)
f_t_mhe = SX.sym('f_t',numHiddenUnits_mhe); % Forget gate
g_t_mhe = SX.sym('g_t',numHiddenUnits_mhe); % Cell candidate
o_t_mhe = SX.sym('o_t',numHiddenUnits_mhe); % Output gate

%% Adding state noises
w_Tw = SX.sym('w_Tw');
w_Tr = SX.sym('w_Tr');
w_ht = SX.sym('w_ht',numHiddenUnits_mhe);
w_ct = SX.sym('w_ct',numHiddenUnits_mhe);

sym_w = vertcat(w_Tw, w_Tr);

%% Controls as parameters - new

% Desired torque acceleration output of electrical machine ( rad/s ) 
M_EM_acc = SX.sym('M_EM_acc', 1);
% Desired torque braking output of electrical machine ( rad/s ) 
M_EM_brk = SX.sym('M_EM_brk', 1);
% Desired torque braking output of friction brake ( rad/s )
M_fric_brk = SX.sym('M_fric_brk', 1);
% % Desired torque acceleration output of electrical machine
% dt_M_EM_acc = u(1);
% % Desired torque braking output of electrical machine 
% dt_M_EM_brk = u(2);
% % Desired torque braking output of friction brake
% dt_M_fric_brk = u(3);

%% Old Parameters
% Grade of the road / track 
phi = SX.sym('phi', 1);
%v_r = SX.sym('v_r',1 );
p = vertcat(M_EM_acc, M_EM_brk, M_fric_brk, phi, v); %, v_r); excluded v_r at the moment coz h is not used
