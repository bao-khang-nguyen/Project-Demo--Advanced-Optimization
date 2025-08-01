% Genetic Algorithm (GA) for PID optimization of buck converter

clc;
clear all; 
close all;
close(findall(0,'Name','GA Vout Responses'));  % Close old plot windows before starting fresh

PopulationSize = 10;       % GA population size
MaxGenerations = 50;       % Maximum number of GA generations

% Define search bounds around known good PID values
lb = [0.005, 4.0, 0.0];    % Lower bounds: Kp, Ki, Kd
ub = [0.02, 6.0, 0.01];    % Upper bounds: Kp, Ki, Kd

% % Optional: Uncomment to fully bias GA to start at known good PID
% initPop = repmat([0.01, 5, 0], 10, 1);

% Generate 9 random individuals (rows) within bounds
initPop = rand(9, 3) .* (ub - lb) + lb;

% Manually add 1 biased individual to guide GA toward known good region
% This individual corresponds to Kp=0.01, Ki=5, Kd=0
biasedIndividual = [0.01, 5, 0];

% Combine random individuals + biased individual = 10x3 initial population
initPop = [initPop; biasedIndividual];

% Composite cost function handle for GA evaluation
% Uses simBuckCompositeVerbose to compute multi-objective performance
cost_fn = @(x) simBuckCompositeVerbose(x(1), x(2), x(3));

% GA options configuration
opts = optimoptions('ga', ...
    'PopulationSize', PopulationSize, ...
    'InitialPopulationMatrix', initPop, ...
    'MaxGenerations', MaxGenerations, ...
    'Display', 'iter', ...         % Show GA progress in the command window
    'OutputFcn', @gaLogger);       % Optional: custom GA logger

% Run GA optimization for 3 variables [Kp, Ki, Kd]
[x_opt, fval] = ga(cost_fn, 3, [], [], [], [], lb, ub, [], opts);

% Display final optimized PID gains
fprintf('\nOptimized Kp = %.4f, Ki = %.4f, Kd = %.4f\n', x_opt(1), x_opt(2), x_opt(3));

%% Simulate with the best PID values from GA
Kp = x_opt(1);
Ki = x_opt(2);
Kd = x_opt(3);

% Assign PID gains to base workspace for Simulink model
assignin('base','Kp',Kp);
assignin('base','Ki',Ki);
assignin('base','Kd',Kd);

% Run Simulink model 'buck_model2' to evaluate the optimized PID
simOut = sim('buck_model2');
ts = simOut.logsout.getElement('Vout').Values.Time;
Vout = simOut.logsout.getElement('Vout').Values.Data;

% Plot best PID response (voltage output over time)
figure('Name','Best PID Vout Response');
plot(ts, Vout, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('V_{out} (V)');
title(sprintf('Best PID Response: Kp=%.4f, Ki=%.4f, Kd=%.4f', Kp, Ki, Kd));
legend('V_{out}');

% Plot GA convergence curve (best cost per generation)
figure('Name','GA Convergence Curve');
plot(1:length(bestCosts), bestCosts, 'b-o');
xlabel('Generation');
ylabel('Best Cost');
title('GA Convergence Curve');
grid on;
