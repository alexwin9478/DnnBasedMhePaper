function [] = codegen_mhe(userpath, pathstr_mhe, ocp)
%% set path
cd([userpath, pathstr_mhe]);

%% Generate C Code
fprintf('Rendering c code ...');
ocp.generate_c_code % start acados code generation
fprintf('Rendering c code done!\n\n');

%% Compile Sfunctions
cd([userpath, pathstr_mhe, '\c_generated_code']);
disp('generating s fun ...');
make_sfun; % ocp solver
disp('S Functions created');

end

