%% Seminar Electromobility--Dynamic Programming (only for NEDC)
% Krunalkumar Zadafiya - 415657
% Aditya Sunil Suryawanshi - 415776
% Mohammed Nawaz Shaikh - 416124 

%% Dynamic programming ~ NEDC(New European Driving Cycle)
tic;
%loading driving cycle
load NEDC_cycle_input.mat

% Creating grid
clear grd
grd.Nx{1}    = 101;         % Number of discrete value for SOC
grd.Xn{1}.hi = 0.95;        % SOC upper limit
grd.Xn{1}.lo = 0.15;        % SOC lower limit

grd.Nu{1}    = 20001;       % Number of discrete value for split ratio
grd.Un{1}.hi = 1;           % Split-ratio upper limit
grd.Un{1}.lo = -1;          % Split-ratio lower limit

% set initial state
grd.X0{1} = 0.50;           % SOC init

% final state constraints
grd.XN{1}.hi = 0.51;        % final SOC range 
grd.XN{1}.lo = 0.501;      % final SOC range

% defining optimization paramters
clear prb

prb.W{1} = w_MGB_NEDC';     %Angular speed of MGB for NEDC
prb.W{2} = T_MGB_NEDC';     %Torque of MGB for NEDC
prb.W{3} = dw_MGB_NEDC';    %Angular acceleration of MGB for NEDC

prb.Ts = 1;                 %Time-step
prb.N  = 1220*1/prb.Ts + 1; %Cycle size

% set options
options = dpm();
options.MyInf = 1e1;
options.BoundaryMethod = 'Line';    % boundary condition 'Line'.
if strcmp(options.BoundaryMethod,'Line') 
    %these options are only needed if 'Line' is used
    options.Iter = 9;
    options.Tol = 1e-8;
    options.FixedGrid = 0;
end

%funcation call to DPM
[res, dyn] = dpm(@hev_qss,[],grd,prb,options);

toc;
run_time = toc - tic;       % total running time of program

%% saving the results for NEDC cycle
save NEDC_outputs.mat res dyn

%% Torque split ratio
% Run this section to see the results in simulink

load NEDC_outputs.mat
load NEDC_cycle_input.mat

% calculate the Split-ratio
u = res.Tm./T_MGB_NEDC';
u(isnan(u)) = 0;            % In case of zero torque, 'u' sets to zero   

% save the split ratio
save split_NEDC_3.mat u

% Below given parameters helps to pass the split-ratio in 
% qss_hybrid_electric_vehicle_example.mdl file
u_in = u;
time = 0:1:1220; 
table1 = table(time', u_in');
file_name= 'split_done.xlsx';
writetable(table1, file_name)
save split_done.mat u_in time;  % Split-ratio stored in file with respected time
