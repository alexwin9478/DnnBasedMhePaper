%% BOF, Description
% Author: Pranav Shah
% This code defines all parameter requried for setting up and compiling a
% Moving Horizon Estimator using acados framework

%% Path

pathstr_mhe = '\dnnbasedmpcmhepmsm\d_MHE';
cd([userpath, pathstr_mhe]);

%% Initialization

%init_modvehicle_IPG;
init_MHE_LSTM;  % Initialize constants for MHE and options P, T, etc.

% Define model name and structure
model_name_mhe = 'LSTM_v21_MHE';  
model_mhe = ocp_MHE_model_NMPC_v8_LSTM;  

%% Arguments for Solver

% Compilation and code generation settings
compile_interface = 'auto';  
codgen_model = 'true';  

% Simulation method (Integrator type)
sim_method = 'discrete';  % Options: 'erk', 'irk', 'irk_gnsf'

%% NLP Solver Settings

nlp_solver = 'sqp';  % Nonlinear programming solver
nlp_solver_exact_hessian = 'false';  
regularize_method = 'no_regularize';  % Options: 'project_reduc_hess'
nlp_solver_max_iter = 5;  % Maximum iterations (e.g., options_mhe.sqp_steps)
nlp_solver_ext_qp_res = 0;  % External QP residual computation

%% QP Solver Settings

qp_solver = 'partial_condensing_hpipm';  % Quadratic programming solver
qp_solver_cond_N = 5;  % Condensing horizon length
qp_solver_cond_ric_alg = 0;  % Factorize Hessian in condensing (0: No, 1: Yes)
qp_solver_ric_alg = 0;  % Riccati algorithm setting (default: 0)
qp_solver_warm_start = 2;  % Warm start strategy (0: Cold, 1: Primal warm start, 2: Primal & dual warm start)
qp_solver_max_iter = 100;  % Maximum iterations for QP solver
qp_solver_print_level = 2;  % Print level for debugging
qp_solver_tol = 1e-4;  % Solver tolerance

% Parameterization scheme (Uncomment if needed)
% param_scheme = 'multiple_shooting';  % Alternative: 'single_shooting'

% Alternative simulation methods (Uncomment if needed)
% sim_method = 'erk';  
% sim_method = 'irk';  

%% Horizon and Cost Function

N = options_mhe.P;  % Horizon length  
cost_type = 'linear_ls';  % Cost function type (Alternative: 'nonlinear_ls')

%% Dims

dims_mhe.Ts = options_mhe.Ts;       % sampling time 0.01
dims_mhe.T = N*dims_mhe.Ts;         % horizon length time [s]
dims_mhe.ny = 4;
dims_mhe.nu = options_mhe.n_controls;
dims_mhe.nx = options_mhe.n_states;
dims_mhe.ny_e = 0;

%% Dimensions Configuration for MHE

% Sampling time
dims_mhe.Ts = options_mhe.Ts;  % Sampling time [s] (e.g., 0.01s)
dims_mhe.T = N * dims_mhe.Ts;   % Horizon length in time [s]
dims_mhe.ny = 4;                % Number of measurements (outputs)
dims_mhe.nu = options_mhe.n_controls;  % Number of control inputs
dims_mhe.nx = options_mhe.n_states;  % Number of states
dims_mhe.ny_e = 0;              % Number of terminal outputs

%% Cost Function Definition for Linear Least Squares

% Number of outputs
nout = 4;   % T_w, T_r, w_Tw, w_Tr
nout_0 = 6; % T_w, T_r, w_Tw, w_Tr, t_w, t_r

% State-to-Output Matrix in Lagrange Term
Vx_mhe = zeros(dims_mhe.ny, dims_mhe.nx);  
Vx_mhe(1, 1) = 1;  
Vx_mhe(2, 2) = 1;  

% State-to-Output Mapping for Arrival Cost Term
Vx_0_mhe = zeros(nout_0, dims_mhe.nx);  
Vx_0_mhe(1, 1) = 1.0;  
Vx_0_mhe(2, 2) = 1.0;  
Vx_0_mhe(nout + 1, 1) = 1.0;  
Vx_0_mhe(nout + 2, 2) = 1.0;  

% Control-to-Output Mapping
% Mapping matrix with 2 controls
Vu_mhe = zeros(dims_mhe.ny, dims_mhe.nu);  
Vu_mhe(3, 1) = 1;  
Vu_mhe(4, 2) = 1;  

% Initial control-to-output mapping
Vu_0_mhe = zeros(nout_0, dims_mhe.nu);  
Vu_0_mhe(3, 1) = 1;  
Vu_0_mhe(4, 2) = 1; 

%% Weigting Matrices

% Arrival cost weighting matrix
Q0_mhe = 10 * diag([0.1, 0.1]);  

% State weighting matrix
Q_mhe = 0.1 * diag([0.2, 0.2]);  

% Control weighting matrix (Noise contribution)
mid = ones(2, 1);  % Placeholder for number of controls
R_mhe = 0.7 * diag(mid);

%% Constraints Definition

% State Bounds
Jbx_mhe = eye(dims_mhe.nx, dims_mhe.nx);  
lbx_mhe = [T_w_min; T_r_min; -2 * ones(2 * numHiddenUnits_mhe, 1)];  
ubx_mhe = [T_w_max + 5; T_r_max + 5; 2 * ones(2 * numHiddenUnits_mhe, 1)];  

% Control Bounds
Jbu_mhe = eye(dims_mhe.nu, dims_mhe.nu);  
lbu_mhe = [0; 0];  
ubu_mhe = [100; 100]; 

%% Setting up the estimator
if skip_mhe_ocp_generation == 0 

    % Create the Model
    ocp_mhe_model = acados_ocp_model();
    ocp_mhe_model.set('name', model_name_mhe);
    ocp_mhe_model.set('T', dims_mhe.T);

    % Define Symbolic Variables
    ocp_mhe_model.set('sym_x', model_mhe.sym_x);
    ocp_mhe_model.set('sym_u', model_mhe.sym_u);
    ocp_mhe_model.set('sym_p', model_mhe.sym_p);

    % Cost Function Setup
    ocp_mhe_model.set('cost_type', cost_type);
    ocp_mhe_model.set('cost_type_e', 'linear_ls'); % Terminal cost type
    ocp_mhe_model.set('cost_type_0', cost_type);

    % Define Cost Weights
    W_0_mhe = blkdiag(Q_mhe, R_mhe, Q0_mhe);
    ocp_mhe_model.set('cost_W_0', W_0_mhe);
    
    W_mhe = blkdiag(Q_mhe, R_mhe);
    ocp_mhe_model.set('cost_W', W_mhe);

    % Set Cost Function Expressions
    if (strcmp(cost_type, 'linear_ls'))
        ocp_mhe_model.set('cost_Vu', Vu_mhe);
        ocp_mhe_model.set('cost_Vx', Vx_mhe);
        ocp_mhe_model.set('cost_Vx_0', Vx_0_mhe);
        ocp_mhe_model.set('cost_Vu_0', Vu_0_mhe);
    else % Nonlinear least squares
        ocp_mhe_model.set('cost_expr_y', model_mhe.expr_y);
        ocp_mhe_model.set('cost_expr_y_0', model_mhe.expr_y_0);
    end

    % Dynamics Definition
    ocp_mhe_model.set('dyn_type', 'discrete');
    ocp_mhe_model.set('dyn_expr_phi', model_mhe.expr_phi);

    % Constraints Setup
    ocp_mhe_model.set('constr_Jbx', Jbx_mhe);
    ocp_mhe_model.set('constr_lbx', lbx_mhe);
    ocp_mhe_model.set('constr_ubx', ubx_mhe);

    ocp_mhe_model.set('constr_Jbu', Jbu_mhe);
    ocp_mhe_model.set('constr_lbu', lbu_mhe);
    ocp_mhe_model.set('constr_ubu', ubu_mhe);

    % Reference Values for Cost
    yref = zeros(nout, 1);
    yref_0 = zeros(nout_0, 1);
    yref_e = zeros(dims_mhe.ny_e, 1);

    ocp_mhe_model.set('cost_y_ref', yref);
    ocp_mhe_model.set('cost_y_ref_e', yref_e);
    ocp_mhe_model.set('cost_y_ref_0', yref_0);

    ocp_mhe_model.set('constr_x0', options_mhe.x0);

    % Display Model Information
    disp('ocp_mhe');
    disp(ocp_mhe_model.model_struct);


    %% Acados OCP Options
    ocp_mhe_opts = acados_ocp_opts();

    ocp_mhe_opts.set('compile_interface', compile_interface);
    ocp_mhe_opts.set('codgen_model', codgen_model);
    ocp_mhe_opts.set('param_scheme_N', N);
    ocp_mhe_opts.set('regularize_method', regularize_method);

    % QP Solver Settings
    ocp_mhe_opts.set('qp_solver', qp_solver);
    ocp_mhe_opts.set('qp_solver_warm_start', qp_solver_warm_start);
    ocp_mhe_opts.set('qp_solver_iter_max', qp_solver_max_iter);

    if (strcmp(nlp_solver, 'sqp'))
        ocp_mhe_opts.set('nlp_solver_tol_stat', qp_solver_tol);
        ocp_mhe_opts.set('nlp_solver_tol_eq', qp_solver_tol);
        ocp_mhe_opts.set('nlp_solver_tol_ineq', qp_solver_tol);
        ocp_mhe_opts.set('nlp_solver_tol_comp', qp_solver_tol);
    end

    if (~isempty(strfind(qp_solver, 'partial_condensing')))
        ocp_mhe_opts.set('qp_solver_cond_N', qp_solver_cond_N);
    end

    if (strcmp(qp_solver, 'partial_condensing_hpipm'))
        ocp_mhe_opts.set('qp_solver_ric_alg', qp_solver_ric_alg);
    end

    % NLP Solver Settings
    ocp_mhe_opts.set('nlp_solver', nlp_solver);
    ocp_mhe_opts.set('nlp_solver_exact_hessian', nlp_solver_exact_hessian);
    ocp_mhe_opts.set('sim_method', sim_method);

    if (strcmp(nlp_solver, 'sqp'))
        ocp_mhe_opts.set('nlp_solver_max_iter', nlp_solver_max_iter);
    end

    % required if discrete
    % ocp_opts.set('sim_method_num_stages', 2);
    % ocp_opts.set('sim_method_num_steps', 5);
    
    % ocp_opts.set('qp_solver_cond_N', N);
    % ocp_opts.set('print_level', 0);
    % ocp_opts.set('ext_fun_compile_flags', '');
    
    % Output Directory
    ocp_mhe_opts.set('output_dir', fullfile('00_temp', ['build_acados_', date()]));

    %% Simulink Opts
    % get default acados simulink opts saved locally
    simulink_opts = get_acados_simulink_opts();
    
    % overwrite default opts - many more available! check function above
    % Input weighting matrices to simulink block
    simulink_opts.inputs.lbx_0 = 1;
    simulink_opts.inputs.ubx_0 = 1;
    simulink_opts.inputs.cost_W_0 = 1;
    simulink_opts.inputs.cost_W = 1;
    simulink_opts.inputs.x_init = 0;
    
    % simulink_opts.inputs.cost_W_e = 0;
    % simulink_opts.inputs.reset_solver = 0; % reset solver flag
    
    % Output action and state trajectory from simulink block
    simulink_opts.outputs.utraj = 1;
    simulink_opts.outputs.xtraj = 1;
    simulink_opts.outputs.cost_value = 1;
    simulink_opts.outputs.KKT_residuals = 1;
    
    %% OCP Generation
    disp('Creating acados MHE ocp...');
    estimator = acados_ocp(ocp_mhe_model, ocp_mhe_opts, simulink_opts);
    disp('acados MHE ocp created.');
    
    % Set Initial Cost Weight
    estimator.set('cost_W', W_0_mhe, 0);

end % End of skip_mpc_ocp_generation

%% Code Generation
if skip_mhe_code_generation == 0
    codegen_mhe(userpath, pathstr_mhe, estimator);
end

addpath(genpath([userpath, pathstr_mhe, '\c_generated_code']));

%% EOF, return to original directory
cd([userpath, pathstr_mhe, '\..\a_Scripts']);
clear mex;