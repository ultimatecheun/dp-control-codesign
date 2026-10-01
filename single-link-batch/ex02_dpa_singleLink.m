clc;
clear all;
close all;

%% Setup the states and the inputs
X = linspace(0,0.4, 383); % Position
V = linspace(0,0.4, 384); % Velocity
F = linspace(-10,10, 81); % Applied force

% Setup the horizon
Tf = 2;          % 2 seconds
T_ocp = 0.05;    % Temporal discretization step
                 % 11 items preferred for time, so divide Tfinal by 10 to get T_ocp
t  = 0 : T_ocp : Tf;

dpf.states           = {X, V};
dpf.inputs           = {F};
dpf.T_ocp            = T_ocp;
dpf.T_dyn            = 0.01;        % Time step for the dynamic simulation         
dpf.n_horizon        = length(t);
dpf.stage_cost_fn    = @stage_cost_fn;
dpf.terminal_cost_fn = @terminal_cost_fn;

% Initiate and run the solver, do forward tracing for the given initial 
% condition and plot the results
tic                  % Let's see how long it takes
dpf = yadpf_solve(dpf);
dpf = yadpf_trace(dpf, [0 0]); % Initial state: [0 0]

% yadpf_plot(dpf, '-');
% Optional: draw the reachability plot
% yadpf_rplot(dpf, [0.5 0], 0.1);

control_optimal = dpf.u_star{1, 1};
n_X = length(dpf.u_star{1});
t_X = [linspace(0, Tf, n_X)]';
T = t_X;

% Set integration step-size parameter
h = t_X(2)-t_X(1); % Also in seconds (s)

% Set the simulink optimization options
opt = simset('solver','ode5','SrcWorkspace','Current','FixedStep',h);

% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal
% Define input vector for simulation that includes time
TU = [T control_optimal];

%Define the initial conditions
x1_0        = 0;
x2_0        = 0;

% With input trajectory defined in TU, perform simulation of optimal
% response of the single-link manipulator between tStart and Tfinal
simOut = sim('SingleLinkManipulator2', [0 Tf], opt, TU);

% Weighting matrix D will be used to compute the Objective function
% Weighting matrix D will be used to compute the Objective function
w = ones(1,length(simOut.x1(end)));
w(length(w)) = 100;
D = diag(w);
    
% Calculate the objective function value
% Let's avoid using a 'for' loop and encourage computational efficiency
% by computing in vectorized form
f = 0.5*((simOut.x1(end)-0.4)'*D*(simOut.x1(end)-0.4)+(control_optimal-7.63)'*(control_optimal-7.63)); 

%% Visulaizing the results obtained
subplot(3,1,1)
plot(T,simOut.x1,'r','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_1')
grid on
title('Optimal states and control signal')

subplot(3,1,2)
plot(T,simOut.x2,'b','LineWidth',1.5)
xlabel('time(s)')
ylabel('x_2')
grid on

subplot(3,1,3)
plot(T,control_optimal,'m','LineWidth',1.5)
xlabel('time(s)')
ylabel('u')
grid on

runTime = toc/60;

disp('run time (min):')
disp(runTime);

%% The stage cost function
function J = stage_cost_fn(X, F, k, dt)
J = 100*dt*(F{1}-7.63).^2 + 1000*dt*(X{2}-0.4).^2;           % For each stage, multiply by dt
end

%% The terminal cost function
function J = terminal_cost_fn(X)
xf = [0.4 0];

% Control gains
r1 = 100;
r2 = 1000;

J = r1*(X{1}-xf(1)).^2 + r2*(X{2}-xf(2)).^2; % Control it using weight and determine which is more paramount
                                             % Velocity in this case
end