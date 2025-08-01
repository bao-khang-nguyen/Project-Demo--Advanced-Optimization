clc;
clear;
close all;

%% ================== PSO PARAMETERS ==================
nParticles = 10;           % Number of particles in the swarm
nIterations = 50;          % Maximum number of PSO iterations
dim = 3;                   % PID parameters: [Kp, Ki, Kd]

% Search space bounds for [Kp, Ki, Kd]
lb = [0.005, 4.0, 0.0];    % Lower bounds
ub = [0.02, 6.0, 0.01];    % Upper bounds

% PSO hyperparameters
wMax = 0.9;                % Initial inertia weight
wMin = 0.4;                % Final inertia weight
c1 = 1.5;                  % Cognitive component (particle memory)
c2 = 1.5;                  % Social component (swarm influence)

%% ================== INITIALIZATION ==================
% Seed particles around known good PID values [0.01, 5, 0]
seed = [0.01, 5, 0];       
spread = 0.2 * (ub - lb);  % 20% of search space as random spread

pos = repmat(seed, nParticles, 1) + rand(nParticles, dim) .* spread; 
pos = max(pos, lb);  % Clip to lower bounds
pos = min(pos, ub);  % Clip to upper bounds

% Initialize velocities
vel = zeros(nParticles, dim);

% Personal best positions and costs
pBest = pos;
pBestCost = inf(nParticles, 1);

% Global best
gBest = zeros(1, dim);
gBestCost = inf;

% Logging
bestCosts = zeros(nIterations, 1);

% Maximum velocity to prevent "particle explosion"
vMax = 0.3 * (ub - lb);

% Create real-time convergence plot
figure('Name','PSO Convergence','NumberTitle','off');
hLine = plot(NaN,NaN,'ro-','LineWidth',1.5);
xlabel('Iteration'); ylabel('Best Cost'); grid on;
title('PSO Convergence Curve');
xlim([1 nIterations]);

%% ================== PSO MAIN LOOP ==================
for iter = 1:nIterations
    for i = 1:nParticles
        % --- Evaluate cost function for each particle ---
        % This runs Simulink simulation and computes the cost
        J = simBuckCompositeVerbose(pos(i,1), pos(i,2), pos(i,3));
        
        % --- Update personal best ---
        if J < pBestCost(i)
            pBest(i,:) = pos(i,:);
            pBestCost(i) = J;
        end
        
        % --- Update global best ---
        if J < gBestCost
            gBest = pos(i,:);
            gBestCost = J;
        end
    end
    
    % --- Update inertia weight (linearly decreasing) ---
    w = wMax - (wMax - wMin) * (iter / nIterations);
    
    for i = 1:nParticles
        r1 = rand(1, dim);
        r2 = rand(1, dim);
        
        % --- PSO velocity update ---
        vel(i,:) = w*vel(i,:) ...
                 + c1*r1.*(pBest(i,:) - pos(i,:)) ...
                 + c2*r2.*(gBest - pos(i,:));
        
        % --- Adaptive mutation to escape local minima ---
        mutation = 0.01 * (1 - iter/nIterations) * randn(1, dim);
        vel(i,:) = vel(i,:) + mutation;
        
        % --- Velocity clamping ---
        vel(i,:) = max(min(vel(i,:), vMax), -vMax);
        
        % --- Position update ---
        pos(i,:) = pos(i,:) + vel(i,:);
        
        % --- Boundary handling ---
        pos(i,:) = max(pos(i,:), lb);
        pos(i,:) = min(pos(i,:), ub);
    end
    
    % --- Log best cost of this iteration ---
    bestCosts(iter) = gBestCost;
    
    % --- Print iteration info ---
    fprintf('Iter %d: Best Cost = %.4f | Kp=%.4f, Ki=%.4f, Kd=%.4f\n', ...
            iter, gBestCost, gBest(1), gBest(2), gBest(3));
    
    % --- Update convergence plot in real-time ---
    set(hLine,'XData',1:iter,'YData',bestCosts(1:iter));
    drawnow;
end

%% ================== FINAL SIMULATION ==================
% Best PID gains found by PSO
Kp = gBest(1);
Ki = gBest(2);
Kd = gBest(3);

% Assign to base workspace for Simulink model
assignin('base','Kp',Kp);
assignin('base','Ki',Ki);
assignin('base','Kd',Kd);

% Run final simulation
simOut = sim('buck_model2');
ts = simOut.logsout.getElement('Vout').Values.Time;
Vout = simOut.logsout.getElement('Vout').Values.Data;

% --- Plot best PID response ---
figure('Name','Best PID Vout Response','NumberTitle','off');
plot(ts, Vout, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('V_{out} (V)');
title(sprintf('Best PID Response: Kp=%.4f, Ki=%.4f, Kd=%.4f', Kp, Ki, Kd));
legend('V_{out}');

% --- Plot final convergence curve ---
figure('Name','PSO Convergence Curve','NumberTitle','off');
plot(1:nIterations, bestCosts, 'ro-','LineWidth',1.5);
xlabel('Iteration');
ylabel('Best Cost');
title('PSO Convergence Curve');
grid on;
