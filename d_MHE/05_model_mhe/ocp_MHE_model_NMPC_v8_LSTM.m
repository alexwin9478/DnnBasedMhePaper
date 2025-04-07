%% BOF
function model_mhe = ocp_MHE_model_NMPC_v8_LSTM()

% import casadi.*
init_modvehicle_IPG;            %Init Vehicle parameters
init_MHE_LSTM;                  %Init constants MHE and Options P,T, etc

% Normalization parameters
n_em_max = 15000; % [rpm]
T_max = 160; % Celsius degree
T_cool = 60; % Celsius degree
T_range = T_max-T_cool; 


%% define the symbolic variables of the plant
ocp_MHE_S02_DefACADOSVarSpace_ac_v8_LSTM;

%% define ode rhs in explicit form (22 equations)
ocp_MHE_S04_SetupNonlinearStateSpaceDynamics_ac_v8_LSTM;

%% Generate casadi C functions
nx = options_mhe.n_states;
nu = options_mhe.n_controls;
np = options_mhe.n_parameter;

%% Populate structure

model_mhe.nx = nx;
model_mhe.nu = nu;
model_mhe.np = np;
model_mhe.sym_x = x;
model_mhe.sym_u = sym_w;
model_mhe.sym_p = p;
model_mhe.expr_phi = expr_phi;
model_mhe.expr_y = y;
model_mhe.expr_y_0 = y_0;

% model.expr_y_e = y_e;

%% EOF

