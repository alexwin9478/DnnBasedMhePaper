 function varargout = setpath(isSubmodule)
% 
%   Signature   : varargout = setpath_nmpc_algebraic(isSubmodule)
%
%   Description : Sets/ removes all necessary paths for project.
%                 1. If project is used as a submodule within another
%                    project, only pass forward the paths to include
%                 2. Checks if a file with include paths already exists in
%                    the temp folder. If not, the last session was ended 
%                    properly and the environment was closed. In this case,
%                    set the environment.
%                 3. Otherwise or Matlab was closed without closing the 
%                    environemnt (the process id will be different) or the
%                    environment is supposed to be closed. In the first 
%                    case, set environment normally. In the latter case 
%                    clear path.
%
%   Parameters  : isSubmodule -> Boolean controlling whether to forward
%                                include paths to calling function
%
%   Return      : varargout -> If project is used as a submodule, output
%                              paths to include
%
%-------------------------------------------------------------------------%

% Get current folder dir
[projectRootDir,~,~] = fileparts(mfilename('fullpath')); 
% fprintf('projectRootDir: %s' , projectRootDir);
% ParentFolder = fullfile(fileparts(mfilename('fullpath')), '..');

idcs = strfind(projectRootDir,'\'); % find \ string
ParentFolder = projectRootDir(1:idcs(end)-1); % get parent directory
% fprintf('\nParent Folder: %s', ParentFolder);

% cd(ParentFolder)
% ParentFolder= pwd;
% cd(projectRootDir);

% Specify required paths
includePaths = {projectRootDir, ...
                genpath([projectRootDir, '\c_MPC\01_init']), ...
                genpath([projectRootDir, '\c_MPC\02_parameter']), ....
                genpath([projectRootDir, '\c_MPC\03_utilities']), ....
                genpath([projectRootDir, '\c_MPC\04_s-functions_em\DENSO_new_001s']), ....
                genpath([projectRootDir, '\c_MPC\05_model']), ...
                genpath([projectRootDir, '\a_Scripts']), ...
                genpath([ParentFolder, '\acados\external\casadi-matlab']),...
                genpath([ParentFolder, '\acados\include']),...
                genpath([ParentFolder, '\acados\lib']),...
                genpath([projectRootDir, '\d_MHE\01_init']),...
                genpath([projectRootDir, '\d_MHE\02_parameter']),...
                genpath([projectRootDir, '\d_MHE\03_utilities']),...
                genpath([projectRootDir, '\d_MHE\05_model_mhe']),...
                genpath([projectRootDir, '\b_model\01_init']),...
                genpath([projectRootDir, '\b_model\02_parameter']),...
                genpath([projectRootDir, '\e_sim'])}.';

% Check which case is active
if nargin > 0 && isSubmodule > 0
    do = 'forward';
else
    % Define path for standard temp dir
    tempFolder_MPC = fullfile(projectRootDir, 'c_MPC\00_temp');
    tempFolder_MHE = fullfile(projectRootDir, 'd_MHE\00_temp');

    % Append temp folder to include path
    includePaths{end + 1} = tempFolder_MPC;
    includePaths{end + 1} = tempFolder_MHE;
    
    % Check if there is a stored file from current or last session
    if ~isfile('includePaths.mat')
        do = 'open';
    else
        
        storedInfo = load('includePaths.mat', 'includePaths', ...
                          'currentPid');
        if storedInfo.currentPid == feature('getpid')
            do = 'close';
        else
            do = 'open';
        end
    end
end

switch do
    case 'forward'
        varargout{1} = includePaths;
        
    case 'open'
        % Create temp folder if non existent
        if ~isfolder(fullfile(tempFolder_MPC))
            mkdir(tempFolder_MPC)
        end
        if ~isfolder(fullfile(tempFolder_MHE))
            mkdir(tempFolder_MHE)
        end
        
        % Add specified folders to Matlab path
        for ii = 1:size(includePaths,1)
            addpath(includePaths{ii});
        end
        
%         % Set work directory for compiled and temporary data
%         Simulink.fileGenControl('set', 'CacheFolder', tempFolder, ...
%                                 'CodeGenFolder', tempFolder);
        
        % Store include path array and current process ID
        currentPid = feature('getpid');
        save('includePaths.mat', 'includePaths', ...
             'currentPid');
    
    case 'close'
        for ii = 1:size(storedInfo.includePaths, 1)
            rmpath(storedInfo.includePaths{ii});
        end
        delete('includePaths.mat')
end