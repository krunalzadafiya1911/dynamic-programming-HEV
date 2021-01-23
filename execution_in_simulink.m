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
