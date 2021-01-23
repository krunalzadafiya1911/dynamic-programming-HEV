function Engine_on_off = controller_dpm_KAN(input)
%'Engine_on_off' function defines the engine state (Either on and off)

% assigning the parameters separately
T_MGB = input(1);       % Torque of MGB
u     = input(2);       % Split-ratio from the dynamic programming

if ((T_MGB<0) && (u==1))
    %Engine state off, i.e., for Electric drive and Regenrative braking
    Engine_on_off = 0;
else
    %Engine state on, i.e., LPS in motor and generator mode and engine drive
    Engine_on_off = 1;
end

%function ends
end