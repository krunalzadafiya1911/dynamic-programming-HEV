function [X, C, I, out] = hev_qss(inp,~)
%function [X C I out] = hev_qss(inp,par)
%HEV_QSS Computes the resulting state-of-charge based on current state-
%   of-charge, inputs and drive cycle demand.
%   
%   [X C I out] = HEV_QSS(INP,PAR)
%
%   INP   = input structure
%   PAR   = user defined parameters
%
%   X     = resulting state-of-charge (battery charge percentage)
%   C     = cost matrix (fuel consumption)
%   I     = infeasible matrix (system infeasibility)
%   out   = user defined output signals (Torque of engine and motor)

%% Intialization

% assigning the individual speed and acceleration values
wg = inp.W{1};
Ttot = inp.W{2};           

% Engine
load OM_622.mat;                %loading the engine parameter
w_MGB = wg;                     % Engine angular velocity
dw_MGB = inp.W{3};              % Engine angular acceleration
theta_CE = 0.2;
w_CE_idle = 105;
T_CE_cutoff = 5;
w_CE_upper = max(w_CE_max);

% Electric Motor
load EM;                        %loading motor parameters
P_aux = 0;
theta_EM = 0.1;
w_EM_upper = max(w_EM_max);


% Battery
load BT;                        %loading battery parameters          
Q_BT_0      = I_0 * 3600;		% Discharge current in 1 h              [C]
U_BT_0      = c_BT_E1+c_BT_E3;  % Mean battery voltage                  [V]
I_BT_max    = (60/t_ch)*I_0;	% Maximum charge/discharge current      [A]
Q_BT_IC = inp.X{1}*3600*I_0;    % Initial battery charge                [C]

% Dynamic parameters
u1 =(Ttot>0).*(inp.U{1}<1);     %engine start-stop
u2 = inp.U{1};                  %split ratio

%% Load simulink blocks
load_system( 'torque_split' )
configObj = Simulink.ConfigSet;
set_param(configObj,'StartTime','0')
set_param(configObj,'StopTime',num2str(size(Ttot,1)))   %stoptime is 1.
set_param(configObj, 'SolverType', 'Fixed-step')
set_param(configObj,'FixedStep', '1')

%% Propagation
% All calulations are done with the simulink file.

% Torque is splitting from here 
% required parameter for simulink calculations
Ttot_in = Ttot;

% m and n are grd.Nx{1} and grd.Nu{1} parameters repectively (given in
% hev_qss_main.m). 
% Assigning it randomly.

m=101;              %grd.Nx{1}...Do not change.
n=2001;            %grd.Nu{1}...Do not change. converged from intialized data

% Problem: Calculating data from the simulink takes too much time (around 1 to 2 days).
% Result: My try is also to reduce it. I reduced it upto 3 hours.
% Aim of Strategy: 1) Minimize the number of calls to torque_split.mdl 2) while
% backward calulation, below given condition helps to reduce the time by
% large difference.

if (size(u1,1)~=1)
    % It executes only for backward calulation, and reduce the number
    %of calls to simulink.

% Controller
    % inputs for simulink file 
    u1_in =  reshape(u1, 1,[]);  %reshape the m*n matrix in 1*mn matrix.
    u2_in = reshape(u2, 1, []);  % this reduce the number of call by m times.
    Q_BT_in = reshape(Q_BT_IC, 1, []);

    % run simulation    
    outputs = sim('torque_split', 'SrcWorkspace','current');

    % torque of engine
    Te_out = outputs.Te_out(1,:);

    % Fuel power consumption
    Tm_out = outputs.Tm_out(1,:);

    % Calculate infeasible
    inps = ~((~((Ttot_in>0)&(u1_in==0))|(Tm_out==Ttot_in))&(~(Ttot_in<=0)|(Tm_out>=(Ttot_in))));
    inps = reshape(inps, [m n]);        %reshape it back in origional size

    Tm_out = reshape(Tm_out, [m n]);    %reshape it back in origional size
    
    % Fuel consumption
    Te_out = reshape(outputs.Te_out(1,:), [m n]);

% Engine
    % Fuel consumption
    Pe_out = reshape(outputs.Pe_out(1,:), [m n]);
    % Calculate infeasible
    ine = (outputs.I2(1,:) | outputs.I3(1,:)); 
    ine = reshape(ine, [m n]);

% EM
    % Calculate infeasible
    inm = (outputs.I4(1,:) | outputs.I5(1,:));
    inm = reshape(inm, [m n]);

% Battery
    % output current
    I_BT_out = reshape(outputs.I_BT(1,:), [m n]);
    % calculate infeasible
    inb = (outputs.I6(1,:) | outputs.I7(1,:));
    inb = reshape(inb, [m n]);

else
    %This will be executed for checking Boundary conditions and Forward
    %propogation
    
    %Here, the inputs are in the shape of 1*m, so the reduction of 
    %execution-time is not possible. Here, reshape command is not involved

% Controller
    % inputs for simulink file in the same way
    u1_in = u1;
    u2_in = u2;
    Q_BT_in = Q_BT_IC;

    % run simulation    
    outputs = sim('torque_split', 'SrcWorkspace','current');

    % Torque of Engine
    Te_out = outputs.Te_out(1,:);

    % Torque of Motor
    %Tm_out(i,:) = outputs.Tm_out(1,:);
    Tm_out = outputs.Tm_out(1,:);

    % Calculate infeasible
    inps = ~((~((Ttot_in>0)&(u1_in==0))|(Tm_out==Ttot_in))&(~(Ttot_in<=0)|(Tm_out>=(Ttot_in))));

% Engine
    % Fuel consumption
    Pe_out = outputs.Pe_out(1,:);
    % Calculate infeasible
    ine = (outputs.I2(1,:) | outputs.I3(1,:)); 

% EM
    % Calculate infeasible
    inm = (outputs.I4(1,:) | outputs.I5(1,:));

% Battery
    % output current
    I_BT_out = outputs.I_BT(1,:);
    % calculate infeasible
    inb = (outputs.I6(1,:) | outputs.I7(1,:));
end

% Summarize infeasible matrix
I = (inps+inb+ine+inm~=0);

% Update State variable
X{1} = (Q_BT_IC - I_BT_out)/(3600*I_0);
X{1} = (conj(X{1})+X{1})/2;

% Calculate cost matrix (fuel mass flow)
C{1}  = Pe_out;

if numel(find(I==0))==0
    % This condition checks for infeasible system.
    keyboard
end

% return
out.Te = Te_out;
out.Tm = Tm_out;
end

%Last change done on 23.03.2021  
%done by Krunalkumar Zadafiya
