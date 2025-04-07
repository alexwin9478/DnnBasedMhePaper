%% Change folder

pathstr = '\dnnbasedmpcmhepmsm\c_MPC';
cd([userpath, pathstr]);

%% Initialization & Data Load

% Initialize vehicle parameters
init_modvehicle_IPG;    

% Initialize MPC and LSTM constants, including options for P, T, etc.
init_MPC_LSTM;          

% Define model name
model_name = 'LSTM_v21'; 

%% Load Race Data - Nordschleife

% Load Nordschleife race data from the specified file
NRND = load('cyc_nordschleife_s_v_666_v130_idiff93_NRND');

% Extract track parameters from the loaded data
s_track     = NRND.data_csv.s_track;             % Track position [m]
v_track     = NRND.data_csv.v_track / mps_kmh;   % Velocity [m/s]
grade_track = NRND.data_csv.grade_track * (180/pi); % Track grade [degrees]
t_track     = NRND.data_csv.t_track;             % Time stamps [s]

%% Driver Model Settings

% Determine the maximum velocity from the track data
v_max = max(v_track);  % Maximum velocity [m/s] (Check unit consistency)

% Set velocity parameters for reference and control
v_max_c = v_max;  % Maximum velocity constraint
v_ref   = v_max;  % Reference velocity [m/s] (Converted from km/h if needed)

%% Arguments Solver & MPC Configuration

% Compilation and code generation settings
compile_interface = 'auto';  
codgen_model = 'true';  

% MPC prediction horizon
N = options.P;  

% NLP Solver Settings
nlp_solver = 'sqp_rti';  % Options: 'sqp'
nlp_solver_exact_hessian = 'false';  
regularize_method = 'no_regularize';  % Options: 'project', 'project_reduc_hess', 'mirror'
nlp_solver_max_iter = options.sqp_steps;  

% Uncomment these lines to set solver tolerances if needed
% ocp_nlp_solver_tol_stat = 1e-8;  
% ocp_nlp_solver_tol_eq   = 1e-8;  
% ocp_nlp_solver_tol_ineq = 1e-8;  
% ocp_nlp_solver_tol_comp = 1e-8;  

% External QP residual computation
nlp_solver_ext_qp_res = 0;  

% QP Solver Settings
qp_solver = 'partial_condensing_hpipm';  % Alternative: 'full_condensing_hpipm', 'full_condensing_qpoases'
qp_solver_cond_N = 5;  
qp_solver_cond_ric_alg = 0;  
qp_solver_ric_alg = 0;  
qp_solver_warm_start = 1;  
qp_solver_max_iter = options.maxIter;  
qp_solver_print_level = 1;  

% Parameterization Scheme (Uncomment if needed)
% param_scheme = 'multiple_shooting';  % Alternative: 'single_shooting'

% Simulation Method
sim_method = 'discrete';  % Options: 'irk', 'erk'

% Simulation Method Steps
if strcmp(sim_method, 'erk')
    sim_method_num_steps = 4;  
else
    sim_method_num_steps = 1;  
end

% Cost Function Type
cost_type = 'linear_ls';  % Alternative: 'nonlinear_ls'


%% Create model

% note: oscillations of controls can be further reduced with state
% augmentation of control, introducing control_rates as controls,
% options are still available in script
model = ocp_model_NMPC_v8_LSTM; % constraint at turning point

%% dims

dims.Ts = options.Ts; % sampling time
dims.T = N*dims.Ts; % horizon length time [s]
dims.nx = model.nx; % 3 + 2 * numHiddenUnits;
dims.nu = model.nu; % 3
dims.ny = length(model.expr_y); % number of outputs in lagrange term y
dims.ny_e = length(model.expr_y_e); % number of outputs in mayer term y_e
dims.nbx = dims.nx; % nx -  2* numHiddenUnits; % bounds
dims.nbx_e = dims.nbx; %
dims.nsbx = 3; %nx;
dims.nsbx_e = 0; %nx; %
dims.nbu = dims.nu;
dims.nsbu = 0;
dims.ng = length(model.expr_g);
dims.ng_e = length(model.expr_g_e);
dims.nsg = dims.ng;
dims.nsg_e = dims.ng_e;
dims.nh = length(model.expr_h); % +1
dims.nh_e = length(model.expr_h_e); % +1
dims.nsh = dims.nh; % +1
dims.nsh_e = 0; %nh_e;
dims.ns = dims.nsh + dims.nsbx + dims.nsbu + dims.ng; % +1
dims.ns_e = dims.nsh_e + dims.nsbx_e + dims.ng_e; % +1
dims.np = model.np; % +1

%% Dimensions Configuration

% Sampling time
dims.Ts = options.Ts;  

% Horizon length in time [s]
dims.T = N * dims.Ts;  

% State and control dimensions
dims.nx = model.nx;  % Number of states (3 + 2 * numHiddenUnits)
dims.nu = model.nu;  % Number of control inputs (3)

% Output dimensions
dims.ny  = length(model.expr_y);   % Number of outputs in Lagrange term y
dims.ny_e = length(model.expr_y_e); % Number of outputs in Mayer term y_e

% State constraints
dims.nbx  = dims.nx;   % Number of state bounds
dims.nbx_e = dims.nbx; % Terminal state bounds

% Slack variables for state constraints
dims.nsbx  = 3;  % Number of slacks for state bounds
dims.nsbx_e = 0; % Terminal state slacks

% Control constraints
dims.nbu  = dims.nu; % Number of control bounds
dims.nsbu = 0;       % Number of slacks for control bounds

% General constraints
dims.ng   = length(model.expr_g);   % Number of general constraints
dims.ng_e = length(model.expr_g_e); % Number of terminal general constraints
dims.nsg  = dims.ng;   % Number of slacks for general constraints
dims.nsg_e = dims.ng_e; % Terminal slacks for general constraints

% Nonlinear inequality constraints
dims.nh   = length(model.expr_h);   % Number of nonlinear constraints
dims.nh_e = length(model.expr_h_e); % Terminal nonlinear constraints
dims.nsh  = dims.nh;   % Number of slacks for nonlinear constraints
dims.nsh_e = 0;        % Terminal slacks for nonlinear constraints

% Total number of slack variables
dims.ns   = dims.nsh + dims.nsbx + dims.nsbu + dims.ng;  
dims.ns_e = dims.nsh_e + dims.nsbx_e + dims.ng_e;  

% Parameter dimensions
dims.np = model.np;  % Number of model parameters


%% cost function mapping
% state-to-output matrix in lagrange term
Vx = zeros(dims.ny, dims.nx);
Vx(1, 3) = 1.0;
% Vx(2, 4) = 1.0;
% Vx(3, 5) = 1.0;
% Vx(4, 6) = 1.0;
% input-to-output matrix in lagrange term
Vu = zeros(dims.ny, dims.nu);
Vu(2, 1) = 1.0;
Vu(3, 2) = 1.0;
Vu(4, 3) = 1.0;
% % state-to-output matrix in mayer term
Vx_e = zeros(dims.ny_e, dims.nx); % EVTL WEG
Vx_e(1, 3) = 1.0;
% Vx_e(2, 4) = 1.0;
% Vx_e(3, 5) = 1.0;
% Vx_e(4, 6) = 1.0;
% NO Vu_e possible!
% weight matrix in lagrange term ----- Q is right now not variable
W = zeros(dims.ny, dims.ny);
% y = [v_1; M_EM_acc; M_EM_brk; M_fric_brk];
W(1, 1) =  Q(3,3); % v
W(2, 2) =  R(1,1); % u1
W(3, 3) =  R(2,2); % u2
W(4, 4) =  R(3,3); % u3
% W(5, 5) =  U(1,1); % dt_u1
% % weight matrix in mayer term
W_e = zeros(dims.ny_e, dims.ny_e); 
W_e(1, 1) =  Q(3,3);
W_e(2, 2) =  R(1,1);
W_e(3, 3) =  R(2,2);
W_e(4, 4) =  R(3,3);
% W_e(5, 5) =  U(1,1);
% output reference in lagrange term / definition later
%yr = ... ;
% output reference in mayer term / definition later
%yr_e = ... ;
% slacks
Z = zeros(dims.ns);
slack_factor = 0.1;% 1~1e-2;
Z(1, 1) = slack_factor* 1; % quadratic, 2er Norm
Z(2, 2) = slack_factor* 1;
Z(3, 3) = slack_factor* 1;
Z(4, 4) = slack_factor* 1;
% Z(5, 5) = slack_factor* 1;
% Z(6, 6) = slack_factor* 1;
% Z(7, 7) = slack_factor* 1;
% Z_e = zeros(ns_e);
% Z_e(1, 1) = slack_factor* 1;
% Z_e(2, 2) = slack_factor* 1;
% Z_e(3, 3) = slack_factor* 1;
% Z_e(4, 4) = slack_factor* 1;
% Z_e(5, 5) = slack_factor* 1;
% Z_e(6, 6) = slack_factor* 1;
% Z_e(7, 7) = slack_factor* 1;

z = 1 * ones(dims.ns,1); %1e2*ones(ns,1); %linear, 1er Norm, set zero
z(1) = z(1); % * S(1,1);
z(2) = z(2); % * S(2,2);
z(3) = z(3); % * S(3,3);
z(4) = z(4); % * S(4,4);

% z(5) = z(5) * S(5,5);
% z(6) = z(6) * S(5,5);
% z(7) = z(7) * S(5,5);
% z_e = 0e5*zeros(ns_e,1);
% z_e(1) = z_e(1) * S(1,1);
% z_e(2) = z_e(2) * S(2,2);
% z_e(3) = z_e(3) * S(3,3);
% z_e(4) = z_e(4) * S(4,4);
% z_e(5) = z_e(5) * S(5,5);
% z_e(6) = z_e(6) * S(5,5);
% z_e(7) = z_e(7) * S(5,5);


%% constraints
%acados_inf = 1e8;
% % state bounds terminal mayer term
% Jbx_e = zeros(nbx, nx);
Jbx_e = eye(dims.nbx, dims.nx);
% Jbx_e(1,1) = 1; 
% Jbx_e(2,2) = 1; 
% Jbx_e(3,3) = 1; 
% lbx_e = [T_w_min; T_r_min; v_min]; % Todo: Impsoe boundaries on all hiden and cell states, change nbx for that! 
% ubx_e = [T_w_max; T_r_max; v_max];
lbx_e = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
ubx_e = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];
% lbx_e = [T_w_min; T_r_min; v_min; M_EM_acc_min; M_EM_brk_min; M_fric_brk_min];
% ubx_e = [T_w_max; T_r_max; v_max; M_EM_acc_max; M_EM_brk_max; M_fric_brk_max];
% state bounds
% Jbx = zeros(nbx, nx);
Jbx = eye(dims.nbx, dims.nx);
% Jbx(1,1) = 1; 
% Jbx(2,2) = 1; 
% Jbx(3,3) = 1;
% lbx = [T_w_min; T_r_min; v_min]; % Todo: Impsoe boundaries on all hiden and cell states, change nbx for that! 
% ubx = [T_w_max; T_r_max; v_max];
lbx = [T_w_min; T_r_min; v_min; -1 * ones(2*numHiddenUnits,1)];
ubx = [T_w_max; T_r_max; v_max; ones(2*numHiddenUnits,1)];
% lbx = [T_w_min; T_r_min; v_min; M_EM_acc_min; M_EM_brk_min; M_fric_brk_min];
% ubx = [T_w_max; T_r_max; v_max; M_EM_acc_max; M_EM_brk_max; M_fric_brk_max];
% input bounds
Jbu = eye(dims.nbu);
lbu = [M_EM_acc_min; M_EM_brk_min; M_fric_brk_min];
ubu = [M_EM_acc_max; M_EM_brk_max; M_fric_brk_max];
% lbu = [dt_M_EM_acc_min; dt_M_EM_acc_min; dt_M_EM_acc_min];
% ubu = [dt_M_EM_acc_max; dt_M_EM_acc_max; dt_M_EM_acc_max];
% nonlinear constraints (power constraint) % S04Line107
lh = [0;0;P_EM_min;0]; %S04_v8
uh = [0;0;P_EM_max;50];
% lh = [0;0;P_EM_min;0]; %S04_v8
% uh = [0;0;P_EM_max;50];
% lh = [0;0;P_EM_min*0]; %S04_v7
% uh = [0;0;P_EM_max*5];
% lh = [0;0];
% uh = [0;0];
% lh_e = [];
% uh_e = [];
% lh_e = [0;0;P_EM_min];
% uh_e = [0;0;P_EM_max];
% linear constraints (power constraint)
% lg = [-1e-4; -1e-4];
% ug = [1e-4; 1e-4];
% lg_e = [-1e-4; -1e-4]; 
% ug_e = [1e-4; 1e-4];
% % soft box state constraints
Jsbx = zeros(19,3);
Jsbx(1, 1) = 1.0;
Jsbx(2, 2) = 1.0;
Jsbx(3, 3) = 1.0;

% Jsbx = eye(nbx, nsbx);
% Jsbx(1, 1) = 1.0;
% Jsbx(2, 2) = 1.0;
% Jsbx(3, 3) = 1.0;
% Jsbx(4, 4) = 1.0;

% soft nonlinear constraints h
Jsh = eye(dims.nh, dims.nsh);
Jsh(1, 1) = 1.0;
Jsh(2, 2) = 1.0;
Jsh(3, 3) = 1.0;
Jsh(4, 4) = 1.0;
% Jsh_e = eye(nh_e, nsh_e);
% Jsh_e(1, 1) = 1.0;
% Jsh_e(2, 2) = 1.0;
% Jsh_e(3, 3) = 1.0;
% % soft linear constraints g
% Jsg = eye(ng, nsg);
% Jsg(1, 1) = 1.0;
% Jsg(2, 2) = 1.0;
% Jsg_e = eye(ng_e, nsg_e);
% Jsg_e(1, 1) = 1.0;
% Jsg_e(2, 2) = 1.0;


%% acados ocp model Definition (Init, Dims, Symbolics, Cost, Dynamics, SetUp OCP)
if skip_mpc_ocp_generation == 0 
ocp_model = acados_ocp_model(); 

ocp_model.set('name', model_name);
% --- dims
ocp_model.set('T', dims.T);
ocp_model.set('dim_nx', dims.nx);
ocp_model.set('dim_nu', dims.nu);
ocp_model.set('dim_ny', dims.ny);
ocp_model.set('dim_ny_e', dims.ny_e);
ocp_model.set('dim_nbx', dims.nbx);
ocp_model.set('dim_nbx_e', dims.nbx_e);
ocp_model.set('dim_nbu', dims.nbu);
ocp_model.set('dim_nh', dims.nh);
% ocp_model.set('dim_nh_e', dims.nh_e);
% ocp_model.set('dim_ng', dims.ng);
% ocp_model.set('dim_ng_e', dims.ng_e);

ocp_model.set('dim_ns', dims.ns);
% ocp_model.set('dim_ns_e', dims.ns_e);
% ocp_model.set('dim_nsbx', dims.nsbx);
% ocp_model.set('dim_nsbx_e', dims.nsbx_e);

% ocp_model.set('dim_nsbu', dims.nsbu);
ocp_model.set('dim_nsh', dims.nsh);
% ocp_model.set('dim_nsh_e', dims.nsh_e);
% ocp_model.set('dim_nsg', dims.nsg);
% ocp_model.set('dim_nsg_e', dims.nsg_e);
ocp_model.set('dim_np', dims.np);

% --- symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('dyn_type', 'discrete');
ocp_model.set('dyn_expr_phi', model.expr_phi);
% ocp_model.set('sym_xdot', model.sym_xdot);
ocp_model.set('sym_p', model.sym_p);

% --- cost
ocp_model.set('cost_type', cost_type);
ocp_model.set('cost_type_e', cost_type);
if (strcmp(cost_type, 'linear_ls'))
	ocp_model.set('cost_Vu', Vu);
	ocp_model.set('cost_Vx', Vx);
	ocp_model.set('cost_Vx_e', Vx_e);
else % nonlinear_ls
	ocp_model.set('cost_expr_y', model.expr_y);
	ocp_model.set('cost_expr_y_e', model.expr_y_e);
end
ocp_model.set('cost_W', W);
ocp_model.set('cost_W_e', W_e);
ocp_model.set('cost_Z', Z);
% ocp_model.set('cost_Z_e', Z_e);
ocp_model.set('cost_z', z);
% ocp_model.set('cost_z_e', z_e);

% --- dynamics
if (strcmp(sim_method, 'erk'))
	ocp_model.set('dyn_type', 'explicit');
	ocp_model.set('dyn_expr_f', model.expr_f_expl);
elseif (strcmp(sim_method, 'irk')) % irk
	ocp_model.set('dyn_type', 'implicit');
	ocp_model.set('dyn_expr_f', model.expr_f_impl);
else
    ocp_model.set('dyn_type', 'discrete');
end

% % state bounds
ocp_model.set('constr_Jbx', Jbx);
ocp_model.set('constr_lbx', lbx);
ocp_model.set('constr_ubx', ubx);
% % input bounds
ocp_model.set('constr_Jbu', Jbu);
ocp_model.set('constr_lbu', lbu);
ocp_model.set('constr_ubu', ubu);
% % nonlinear constraints 
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', lh);
ocp_model.set('constr_uh', uh);
% ocp_model.set('constr_expr_h_e', model.expr_h_e);
% ocp_model.set('constr_lh_e', lh_e);
% ocp_model.set('constr_uh_e', uh_e);
% % linear constraints
% ocp_model.set('constr_expr_g', model.expr_g);
% ocp_model.set('constr_lg', lg);
% ocp_model.set('constr_ug', ug);
% ocp_model.set('constr_expr_g_e', model.expr_g_e);
% ocp_model.set('constr_lg_e', lg_e);
% ocp_model.set('constr_ug_e', ug_e);
% % state bounds terminal
ocp_model.set('constr_Jbx_e', Jbx_e);
ocp_model.set('constr_lbx_e', lbx_e);
ocp_model.set('constr_ubx_e', ubx_e);
% soft nonlinear constraints
ocp_model.set('constr_Jsbx', Jsbx);
% ocp_model.set('constr_Jsbx_e', Jsbx);
ocp_model.set('constr_Jsh', Jsh); 
% ocp_model.set('constr_Jsh_e', Jsh_e);
% % soft linear constraints
% ocp_model.set('constr_Jsg', Jsg);
% ocp_model.set('constr_Jsg_e', Jsg_e);
% (dummy) initial state constr
ocp_model.set('constr_x0', options.x0);
ocp_model.model_struct

% --- acados ocp opts
ocp_opts = acados_ocp_opts();
ocp_opts.set('compile_interface', compile_interface);
ocp_opts.set('codgen_model', codgen_model);
% ocp_opts.set('param_scheme', param_scheme);
ocp_opts.set('param_scheme_N', N);

if (exist('shooting_nodes', 'var'))
	ocp_opts.set('param_scheme_shooting_nodes', shooting_nodes);
end

ocp_opts.set('nlp_solver', nlp_solver);
% if (strcmp(nlp_solver, 'sqp')) % not available for sqp_rti
% ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
% ocp_opts.set('nlp_solver_tol_stat', tol);
% ocp_opts.set('nlp_solver_tol_eq', tol);
% ocp_opts.set('nlp_solver_tol_ineq', tol);
% ocp_opts.set('nlp_solver_tol_comp', tol);
% end
ocp_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
ocp_opts.set('regularize_method', regularize_method);
ocp_opts.set('nlp_solver_ext_qp_res', nlp_solver_ext_qp_res);
if (strcmp(nlp_solver, 'sqp'))
	ocp_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
end
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_iter_max', qp_solver_max_iter);
ocp_opts.set('qp_solver_warm_start', qp_solver_warm_start);
ocp_opts.set('qp_solver_cond_ric_alg', qp_solver_cond_ric_alg);
if (contains(qp_solver, 'partial_condensing'))
	ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
end
if (strcmp(qp_solver, 'partial_condensing_hpipm'))
	ocp_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
end
ocp_opts.set('sim_method', sim_method);
% ocp_opts.set('sim_method_num_stages', sim_method_num_stages);
% ocp_opts.set('sim_method_num_steps', sim_method_num_steps);
ocp_opts.set('print_level', qp_solver_print_level);

%% Simulink Options
% get default acados simulink opts saved locally
simulink_opts = get_acados_simulink_opts();

% overwrite default opts - many more available! check function above
% Input weighting matrices to simulink block
simulink_opts.inputs.cost_W_0 = 0;
simulink_opts.inputs.cost_W = 0;
simulink_opts.inputs.cost_W_e = 0;
simulink_opts.inputs.reset_solver = 0; % reset solver flag

% Output action and state trajectory from simulink block
simulink_opts.outputs.utraj = 0;
simulink_opts.outputs.xtraj = 0;

% --- acados ocp, create ocp
ocp_opts.set('output_dir', fullfile('00_temp', ['build_acados_', date()]));
disp('Creating acados mpc ocp...');
ocp = acados_ocp(ocp_model, ocp_opts, simulink_opts);
disp('acados mpc ocp created.');

end % skip_mpc_ocp_generation


%% Render templated Code for the model contained in ocp object
if skip_mpc_code_generation == 0
    codegen_mpc(userpath, pathstr, ocp); % we could create the Simulink options here and make one common function
end % skip_mpc_code_generation


%% Prepare to run Sim on local machine
cd([userpath, pathstr]);
addpath(genpath([userpath, pathstr, '\c_generated_code']));


%% EOF, return to original directory
cd([userpath, pathstr, '\..\a_Scripts']);
clear mex;