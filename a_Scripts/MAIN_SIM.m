%% BOF

%% put this into the sim scripts, not the main?

clear mex;

import casadi.* % load casadi

[projectRootDir,~,~] = fileparts(mfilename('fullpath'));
idcs = strfind(projectRootDir,'\'); % find \ string
userpath = projectRootDir(1:idcs(end-1)-1); % get parent directory

% check that acados env.sh has been run
env_run = getenv('ENV_RUN');
if (~strcmp(env_run, 'true'))
	disp('env.sh has not been sourced! Before executing this example, we run: source env.sh');
    cd ([userpath '\acados\examples\acados_matlab_octave']);
    acados_env_variables_windows;
    disp('env.sh has been sourced!');
    %cd([userpath, pathstr]);
end


%% Settings OCP-Gen & Code-Gen
% MPC
skip_mpc_ocp_generation      = 0;           % set 1 to skip solver generation
skip_mpc_code_generation        = 0;        % set 1 to skip code generation
if skip_mpc_ocp_generation   == 1
    skip_mpc_code_generation    = 1;
end
% MHE
skip_mhe_ocp_generation      = 0;           % set 1 to skip solver generation
skip_mhe_code_generation        = 0;        % set 1 to skip code generation
if skip_mhe_ocp_generation   == 1
    skip_mhe_code_generation    = 1;
end

%% Generate MPC
Generate_MPC; 


%% Generate MHE
Generate_MHE;


%% Run Sim
SE_selector_sim = 1; % 1: MHE, 2: Denso (Ideal values)
% start_sim_offline;
start_sim_offline_public;
