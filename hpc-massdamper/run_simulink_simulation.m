%% 🚀 **Helper Function for Simulink Simulation**
function [x1_new, x2_new] = run_simulink_simulation(x1_0, x2_0, force_input, h, opt)
    m = 1;   % Mass
    b = 0.1; % Damping coefficient

    % Set integration step-size parameter
    h = 0.1;  % == T_ocp; Also in seconds (s)
    Tf = h;   % Final time

    % Set the simulink optimization options
    opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
    
    
    TU = [0 force_input];

    % Run Simulink model
    simOut = sim('massDamper', [0 h], opt, TU);

    % Extract results
    x1_new = simOut.x1(end);
    x2_new = simOut.x2(end);
end