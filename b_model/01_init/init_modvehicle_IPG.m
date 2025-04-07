%% Vehicle Model Initialization file
% File for initalisation of vehicle model

%% Parameter E-machine
% Load Parameter EM
load('param_maxtorque.mat');


%% Parameter Vehicle
r_dyn = 0.293/0.9624;% (m) tire diameter
grav = 9.81; %m/s^2
dens = 1.2041; %20C %1.1849;%(kg/m^3) @ 25 C density
Cd = 0.355; %0.27 %coefficent of drag
Af = 2.26; %m^2 %statt 2.19; %m^2
fr = 0.011; %coefficent of friction
FDR = 9.3; % 9.3 %final drive ratio
Mv = 1160; %kg 
Pi=pi;
torque_brake = 515;      %Nm brake torque % F_fzg_min = -21.46 kN mit a_min = -18,5 m/s² (aus IPG) und Mv (F_rad = MV * a), M_EM = F_Fzg * r_dyn / FDR
rps_rpm = 30/pi;     % rad/sec to rpm
rpm_rps = pi/30;
mps_kmh = 3.6;
kmh_mps = 1/3.6;

% Driver model paramters
v_max = 130; % 130 km/h
v_max_c = v_max; % making v dimensionless
% keep unchanged (Note: Jinming Liu)
Kf_c = 1/10;
Kp_c = 30;
Ti_c = 60;
Tt_c = 65;