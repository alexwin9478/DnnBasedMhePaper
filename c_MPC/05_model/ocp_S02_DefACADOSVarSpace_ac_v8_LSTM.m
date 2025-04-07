import casadi.*

% States % TODO: change names like ht1 
% Winding / stator temperatur (°C) 
T_w = MX.sym('T_w', 1);
% Rotor temperature (°C) 
T_r = MX.sym('T_w', 1);
% Vehicle speed ( m/s ) 
v = MX.sym('v', 1);
% % PMSM Acc Torque ( Nm )
% M_EM_acc = x(4);
% % PMSM Dec Torque ( Nm )
% M_EM_brk = x(5);
% % Friction Brake Dec Torque ( Nm )
% M_fric_brk = x(6);

ht = MX.sym('ht',numHiddenUnits); % HiddenhHiddeH
ct = MX.sym('ct',numHiddenUnits); % Cell states

% x = vertcat(T_w, T_r, v, M_EM_acc, M_EM_brk, M_Fric_brk, ht1, ct1); % states, add all statee
x = vertcat(T_w, T_r, v, ht, ct); % states, add all statee

%% differential states
v_dot = MX.sym('v_dot', 1); % acceleration, this is discretized in ocp:S04

%% algebraic variables
i_t = MX.sym('i_t',numHiddenUnits); % Input gate (sigmoid function)
f_t = MX.sym('f_t',numHiddenUnits); % Forget gate
g_t = MX.sym('g_t',numHiddenUnits); % Cell candidate
o_t = MX.sym('o_t',numHiddenUnits); % Output gate

%% Control inputs 

% Desired torque acceleration output of electrical machine ( rad/s ) 
M_EM_acc = MX.sym('M_EM_acc', 1);
% Desired torque braking output of electrical machine ( rad/s ) 
M_EM_brk = MX.sym('M_EM_brk', 1);
% Desired torque braking output of friction brake ( rad/s )
M_fric_brk = MX.sym('M_fric_brk', 1);
% % Desired torque acceleration output of electrical machine
% dt_M_EM_acc = u(1);
% % Desired torque braking output of electrical machine 
% dt_M_EM_brk = u(2);
% % Desired torque braking output of friction brake
% dt_M_fric_brk = u(3);

u = vertcat(M_EM_acc, M_EM_brk, M_fric_brk);

%% Parameters
% Grade of the road / track 
phi = MX.sym('phi', 1);
v_r = MX.sym('v_r',1 );
p = vertcat(phi, v_r); % phi (+ T_w_0 + T_r_0)

% T_w_0 = p(2);
% T_r_0 = p(3);