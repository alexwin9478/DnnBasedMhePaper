function [] = codegen_mpc(userpath, pathstr, ocp)
%% set path
cd([userpath, pathstr]);


%% Generate C Code
fprintf('Rendering c code ...');
ocp.generate_c_code; % start acados code generation
fprintf('Rendering c code done!\n\n');

%% Compile Sfunctions
cd([userpath, pathstr, '\c_generated_code']);
disp('generating s fun ...');
make_sfun; % ocp solver
disp('S Functions created');

end

