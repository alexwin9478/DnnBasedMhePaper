%% Mil Main Run File

%% Predefining some constant parameters (check init files in both)
% Normalization parameters
n_em_max = 15000; % [rpm]
T_max = 160; % Celsius degree
T_cool = 60; % Celsius degree
T_range = T_max-T_cool; 

% loading EM efficiency map
load('data_efficiency_TPS_Gl1_60C.mat');

%% Settings Sim (Put into sim? TODo?)
t_end                       = 8000;        % s end time max - Simulink
t_mpc_on                    = 3;           % s start time when MPC Controller starts (Threshold Function vor Ausgang MPC Command) - Simulink
step_size                   = 0.01;        % Step size for simulation - Simulink
start_distance_check        = 0.1;         % Security Check if car is not moving anymore, from how many meters it will start - Simulink
break_criteria              = 0.001;       % Security Check if car is not moving anymore, minimum difference in meters betweend s_0 and s_1 - Simulink
gain_Twr                    = 1.0;         % Tweaking factor on T_w/r after model (to get more derating)  - Simulink


%% MPC Main
% Change start settings for simpler initialization
% v_track(1:128,1) = 1;
% driver model settings
v_max = max(v_track)*0.9952; % Todo? 130 km/h, but driver in IPG is inaccurate and overshoots as well as driver model here
v_max_c = v_max; %making v dimensionless
v_ref = v_max; % todo check?


%% NMPC Variables and States: Declare model variables
% todo: put these in the ANN and MPC inits or read from ocp
options.n_real_states = 3;
options.n_constraints_h = 4; % number of nonlinear additional constraints


%% Init Values
lbg(1:(options.n_states*(options.P+1)+options.P)) = -1e-10;  % 0; % -1e-20  % Equality constraints
ubg(1:(options.n_states*(options.P+1)+options.P)) = 1e-10;  % 0; % 1e-20   % Equality constraints

lbx(1:options.n_states:options.n_states*(options.P+1),1) = T_w_min; % state Tw lower bound % STATES
ubx(1:options.n_states:options.n_states*(options.P+1),1) = T_w_max; % state Tw upper bound
lbx(2:options.n_states:options.n_states*(options.P+1),1) = T_r_min; % state Tr lower bound
ubx(2:options.n_states:options.n_states*(options.P+1),1) = T_r_max; % state Tr upper bound
lbx(3:options.n_states:options.n_states*(options.P+1),1) = v_min; % state v lower bound
ubx(3:options.n_states:options.n_states*(options.P+1),1) = v_max; % state v upper bound
for i = (options.n_real_states+1):options.n_states
    lbx(i:options.n_states:options.n_states*(options.P+1),1) = -1000; % state v lower bound ct and ht
    ubx(i:options.n_states:options.n_states*(options.P+1),1) = 1000; % state v upper bound ct and ht
end
% lbx(4:n_states:n_states*(options.P+1),1) = M_EM_acc_min; %state v lower bound
% ubx(4:n_states:n_states*(options.P+1),1) = M_EM_acc_max; %state v upper bound
% lbx(5:n_states:n_states*(options.P+1),1) = M_EM_brk_min; %state v lower bound
% ubx(5:n_states:n_states*(options.P+1),1) = M_EM_brk_max; %state v upper bound
% lbx(6:n_states:n_states*(options.P+1),1) = M_fric_brk_min; %state v lower bound
% ubx(6:n_states:n_states*(options.P+1),1) = M_fric_brk_max; %state v upper bound
%unten im Vektor
% lbx(n_states*(options.P+1)+1:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_min; %M_EM_acc lower bound % CONTROLS
% ubx(n_states*(options.P+1)+1:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_max; %M_EM_acc upper bound
% lbx(n_states*(options.P+1)+2:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_min; %M_EM_brk lower bound
% ubx(n_states*(options.P+1)+2:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_max; %M_EM_brk upper bound
% lbx(n_states*(options.P+1)+3:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_min; %M_fric_abrk lower bound
% ubx(n_states*(options.P+1)+3:n_controls:n_states*(options.P+1)+n_controls*options.P,1) = dt_M_EM_acc_max; %M_fric_brk upper bound

x0 = [T_w_0 ; T_r_0 ; v_0];    % Todo initial condition.
x0_0 = x0;
x0_0_ac = [x0; zeros(2*numHiddenUnits,1)];
u0 = ones(options.n_controls,1);
u0 = u0 .* [0; 0; 0];
u0 = repmat(u0,1,options.P)'; % control inputs
X0 = repmat(x0,1,options.P)'; % initialization of the states decision variables
X0_ref = repmat(x0(1:3),1,options.P)'; 
lam_x0 = zeros(1,length(lbx));
lam_g0 = zeros(1,length(lbg));
u0_0 = zeros(options.P*options.n_controls,1);

lbx_ac = lbx(1:options.n_states*(options.P-1),1);
lbx_ac_e = lbx(options.n_states*(options.P)+1:options.n_states*(options.P+1),1);
ubx_ac = ubx(1:options.n_states*(options.P-1),1);
ubx_ac_e = ubx(options.n_states*(options.P)+1:options.n_states*(options.P+1),1);

u_min_0 = [M_EM_acc_min; M_EM_brk_min; M_fric_brk_min];
u_min(1:options.n_controls:options.n_controls*(options.P),1) = M_EM_acc_min;
u_min(2:options.n_controls:options.n_controls*(options.P),1) = M_EM_brk_min;
u_min(3:options.n_controls:options.n_controls*(options.P),1) = M_fric_brk_min;

u_max_0 = [M_EM_acc_max; M_EM_brk_max; M_fric_brk_max];
u_max(1:options.n_controls:options.n_controls*(options.P),1) = M_EM_acc_max;
u_max(2:options.n_controls:options.n_controls*(options.P),1) = M_EM_brk_max;
u_max(3:options.n_controls:options.n_controls*(options.P),1) = M_fric_brk_max;

% u_min_0 = [dt_M_EM_acc_min; dt_M_EM_acc_min; dt_M_EM_acc_min];
% u_min(1:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_min;
% u_min(2:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_min;
% u_min(3:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_min;
% 
% u_max_0 = [dt_M_EM_acc_max; dt_M_EM_acc_max; dt_M_EM_acc_max];
% u_max(1:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_max;
% u_max(2:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_max;
% u_max(3:n_controls:n_controls*(options.P),1) = dt_M_EM_acc_max;


lbu = u_min; lbu_0 = lbu;
ubu = u_max; ubu_0 = ubu;
xx0_ac = x0; % x_meas;
y_ref = zeros((options.P-1)*(options.n_outputs),1);
y_ref_0 = y_ref;
y_ref_0_0 = zeros(options.n_outputs,1);
y_ref_e = zeros(options.n_outputs,1);
y_ref_e_0 = y_ref_e;

states_0 = zeros(options.P*options.n_states,1);

phi_0 = [zeros((options.P+1),1)]; % phi / grade_track + v_ref
parameter_0 = [phi_0; zeros(options.P + 1,1)]; % phi, v
lbh_ac(1:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;
lbh_ac(2:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;
lbh_ac(3:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = P_EM_min; % simplification here
lbh_ac(4:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;
ubh_ac(1:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;
ubh_ac(2:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;
ubh_ac(3:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = P_EM_max; % simplification here
ubh_ac(4:options.n_constraints_h:options.n_constraints_h*(options.P-1),1) = 0;

lbg_ac = lbh_ac;
ubg_ac = ubh_ac;

% u_ref = [0];
u_ref = zeros((options.n_controls)*options.P,1);
T_ref = repmat([T_w_ref, T_r_ref], options.P, 1);

disp('Sim Init for MPC done');

% ---------------------------------------------------------------------------------
%% Simulation
open_system('Offline_MiL_Sim_Public');
set_param(gcs, 'EnablePacing', 'off');
set_param(gcs,'SimulationCommand','Update')

tic
sim('Offline_MiL_Sim_Public');
t_sim = toc;
sound(sin(1:8000)); % Acoustic warnining that simulation stopped

%% Print
fprintf('\n ---------------Simulation completed---------------- \n')
fprintf('\n Computation time = %4.1f s \n' ,t_sim );
fprintf('\n vmax-v_grenz_track = %4.2f km/h ' ,(max(v_ist)-max(v_track))*3.6);
fprintf('\n vmax-v_grenz_des = %4.2f km/h' ,(max(v_ist)-max(v_des))*3.6);
fprintf('\n mean(abs(v_ist-v_grenz)) = %4.2f km/h' , mean(abs(v_ist-v_des))*3.6);
fprintf('\n RMSE v_ist = %4.2f km/h \n' , sqrt(mean((v_ist-v_des).^2))*3.6);

fprintf('\n mean(T_w) = %4.2f °C' , mean(T_w));
fprintf('\n mean(T_r) = %4.2f °C' , mean(T_r));
fprintf('\n max(T_w) = %4.2f °C' , max(T_w));
fprintf('\n max(T_r) = %4.2f °C \n' , max(T_r));

fprintf('\n percentage of driven distance: %4.2f percent',100 * Dist(end)/s_track(end));
fprintf('\n Finish time NMPC: %4.2f s',t_finish);
fprintf('\n Finish time no derating IPG: %4.2f s', NRND.data_csv.t_track(end));
t_finish_compare = [694.5; 730.59; 702; 0]; % model: no derating, L125155 Derating Rekup X, best MPC result, X
fprintf('\n Finish time Derating125155: %4.2f s', t_finish_compare(2,1));
fprintf('\n Finish time no derating model: %4.2f s', t_finish_compare(1,1));
fprintf('\n Finish time best NMPC (outdated): %4.2f s', t_finish_compare(3,1));
fprintf('\n Improvement to linear derating = %4.2f s',t_finish_compare(2,1)-t_finish);
fprintf('\n Improvement to best NMPC result = %4.2f s \n',t_finish_compare(3,1)-t_finish);

fprintf('\n Recuperated energy = %4.3f kWh',E_rekup(end));
fprintf('\n Total energy consumption = %4.3f kWh \n',E_total(end));
fprintf('\n Number average iterations = %4.2f', mean(solver_sqp_iter));
fprintf('\n Percentage of Non-Successes of solver = %4.2f' ,100 * length(find(solver_status == 4))/length((solver_status)));
fprintf('\n Percentage of maximum Iterations of solver = %4.2f \n' ,100 * length(find(solver_status == 2))/length((solver_status)));
fprintf('\n Number average Solver CPU Time = %4.3f', mean(solver_cpu_time));
fprintf('\n Number maximum Solver CPU Time = %4.3f', max(solver_cpu_time));
fprintf('\n Standard deviation Solver CPU Time = %4.3f \n', sqrt(mean((solver_cpu_time-mean(solver_cpu_time)).^2)));


%% Store Data
if true 
stor.E_Rekup = E_rekup(end);
stor.E_total = E_total(end);
stor.t_finish = tout(end);
stor.t_sim = t_sim;
stor.p = options.P;
stor.m = options.M;
stor.Ts = options.Ts;
stor.v_des = v_des;
stor.v_ist = v_ist;
stor.v_mean = 3.6*mean(abs(v_des(50:end)-v_ist(50:end)));
stor.v_max = 3.6*max(abs(v_des(50:end)-v_ist(50:end)));
stor.weights_ov_Tw  =  weights.OV(1);
stor.weights_ov_Tr  =  weights.OV(2);
stor.weights_ov_v  =  weights.OV(3);
stor.weights_mv_Macc  =  weights.MV(1);
stor.weights_mv_Mbr  =  weights.MV(2);
stor.weights_mv_Frbr  =  weights.MV(3);
stor.weights_mvrate_Macc  =  weights.MVrate(1);
stor.weights_mvrate_Mbr  =  weights.MVrate(2);
stor.weights_mvrate_Frbr  =  weights.MVrate(3);
stor.weights_ov_Tr  =  weights.OV(2);
stor.weights_ov_v  =  weights.OV(3);
stor.scales_ov_Tw  =  x_scale(1);
stor.scales_ov_Tr  =  x_scale(2);
stor.scales_ov_v  =  x_scale(3);
stor.scales_mv_Macc  =  u_scale(1);
stor.scales_mv_Mbr  =  u_scale(2);
stor.scales_mv_Frbr  =  u_scale(3);
stor.scales_mvrate_Macc  =  urate_scale(1);
stor.scales_mvrate_Mbr  =  urate_scale(2);
stor.scales_mvrate_Frbr  =  urate_scale(3);
stor.Tr = T_r;
stor.Tw = T_w;
stor.maxTw = max(T_w);
stor.maxTr = max(T_r);
stor.endTw = T_w(end);
stor.endTr = T_r(end);
stor.T_w_ref = T_w_ref;
stor.T_r_ref = T_r_ref;
stor.solver_t_end = t_end;
stor.gain_Twr = gain_Twr;

% save data
% save([userpath, pathstr, '\07_results\mpc_', datestr(now,'yyyymmdd_HHMM'),'_P', ...
%     num2str(options.P), '_MaxIter', num2str(options.maxIter),'_', description])
end
