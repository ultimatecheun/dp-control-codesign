function X = simulink_state_update_fn(X, F, dt)
    m = 1;   % Mass
    b = 0.1; % Damping coefficient

    % Set integration step-size parameter
    h = 0.1;  % == T_ocp; Also in seconds (s)
    Tf = h;   % Final time

    % Set the simulink optimization options
    opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);
    
    [p, q] = size(X{1, 1});

    x1_updated = zeros(p, q);
    x2_updated = zeros(p, q);

    % Create a parallel pool
    pool = gcp('nocreate');
    if isempty(pool)
        pool = parpool; % Create a new parallel pool if none exists
    end

    % Attach all necessary files to the pool
    addAttachedFiles(pool, {'run_simulink_simulation.m'});

    % Convert cell arrays to matrices for parallel processing
    X1 = X{1, 1};
    X2 = X{1, 2};
    F1 = F{1, 1};

    parfor j = 1:q
        for i = 1:p
            x1_0 = X1(i, j);
            x2_0 = X2(i, j);
            force_input = F1(i, j);

            % Call a separate function to perform Simulink simulation
            [x1_new, x2_new] = run_simulink_simulation(x1_0, x2_0, force_input, h, opt);

            % Store results
            x1_updated(i, j) = x1_new;
            x2_updated(i, j) = x2_new;
        end
        
        % Simulate the system with sampling period of Tdyn
        fprintf('Simulating the system one step ahead...\n');
        fprintf('Progress: ')
        fprintf('%.6f%% ', (i*j)/(p*q)*100)
    end

    % Delete the parallel pool after execution
    delete(pool);

    % Return updated states
    X{1} = x1_updated;
    X{2} = x2_updated;
end
