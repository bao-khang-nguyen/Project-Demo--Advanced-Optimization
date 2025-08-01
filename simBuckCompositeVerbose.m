function J = simBuckCompositeVerbose(Kp, Ki, Kd)
    persistent hFig;

    % Assign PID gains to base workspace
    assignin('base','Kp',Kp);
    assignin('base','Ki',Ki);
    assignin('base','Kd',Kd);
    assignin('base', 'currentPID', [Kp, Ki, Kd]);

    % Run Simulink model
    simOut = sim('buck_model2');

    % Extract signals
    ts = simOut.logsout.getElement('error').Values.Time;
    e = simOut.logsout.getElement('error').Values.Data;
    Vout = simOut.logsout.getElement('Vout').Values.Data;

    Vref = 5;
    finalVout = Vout(end);
    peak = max(Vout);
    overshoot = max(0, (peak - Vref) / Vref * 100);
    undershoot = min(Vout);

    %% --- Classical Metrics ---
    IAE = sum(abs(e));
    ISE = sum(e.^2);
    ITSE = sum(ts .* e.^2);
    ISTSE = sum(ts.^2 .* e.^2);

    %% --- Rise Time ---
    try
        idx90 = find(Vout >= 0.9 * Vref, 1);
        riseTime = ts(idx90);
    catch
        riseTime = ts(end);  % Fallback if 90% not reached
    end

    %% --- Overshoot Penalty ---
    if overshoot > 2
        overshootPenalty = 1000 * (overshoot);
    else
        overshootPenalty = 0;
    end

    %% --- Undershoot Penalty ---
    if undershoot < 0.9 * Vref
        undershootPenalty = 100 * (Vref - undershoot)^2;
    else
        undershootPenalty = 0;
    end

    %% --- Steady-State Penalty ---
    steadyStateError = abs(finalVout - Vref);
    steadyStatePenalty = min(1e4 * steadyStateError^2, 1e4);  % Max 10k

    %% --- Settling Penalty ---
    try
        within5 = abs(Vout - Vref) <= 0.05 * Vref;
        lastIdx = find(within5, 1, 'last');
        if lastIdx < length(Vout)
            settlingPenalty = 1000;
        else
            settlingPenalty = 0;
        end
    catch
        settlingPenalty = 1000;
    end

%% --- Jerk Penalty for Rise-Then-Drop Behavior (Peak Detection via dV) ---
V = Vout(:);  % Ensure column
dv = diff(V);  % First derivative
jerkPenalty = 0;

% Detect slope reversal: dv(i) > 0 followed by dv(i+1) < 0
for i = 1:length(dv)-1
    if dv(i) > 0 && dv(i+1) < 0
        % Penalize strength of reversal
        jerkPenalty = jerkPenalty + (abs(dv(i)) + abs(dv(i+1)));
    end
end

% Amplify penalty to ensure optimizer avoids it
if jerkPenalty > 0
    jerkPenalty = jerkPenalty * 1000;  % TUNE THIS MULTIPLIER as needed
end


    %% --- Rise Time Penalty ---
    if riseTime > 0.04
        risePenalty = 1e4 * (riseTime - 0.04)^2;
    elseif riseTime < 1e-3
        risePenalty = 1e5;
    else
        risePenalty = 0;
    end

    %% --- Composite Cost Function ---
    J = (1/10000)*IAE + (1/10000)*ISE + 0.005*ITSE + 0.001*ISTSE + ...
        risePenalty + overshootPenalty + undershootPenalty + ...
        jerkPenalty + steadyStatePenalty + settlingPenalty;

    %% --- Assign to base workspace ---
    assignin('base', 'lastIAE', IAE);
    assignin('base', 'lastISE', ISE);
    assignin('base', 'lastITSE', ITSE);
    assignin('base', 'lastISTSE', ISTSE);
    assignin('base', 'lastRisePenalty', riseTime);
    assignin('base', 'lastOvershootPenalty', overshootPenalty);
    assignin('base', 'lastJerkPenalty', jerkPenalty);
    assignin('base', 'lastSettlingPenalty', settlingPenalty);
    assignin('base', 'lastUndershootPenalty', undershootPenalty);
    assignin('base', 'lastSteadyStatePenalty', steadyStatePenalty);
    assignin('base', 'lastOvershoot', overshoot);
    assignin('base', 'lastCost', J);

    %% --- Plot for visual comparison ---
    if isempty(hFig) || ~isvalid(hFig)
        hFig = figure('Name','GA Vout Responses','NumberTitle','off');
        hold on;
        grid on;
        xlabel('Time (s)');
        ylabel('V_{out} (V)');
        title('GA: V_{out} Responses per Candidate');
    end

    figure(hFig);
    plot(ts, Vout, 'Color', rand(1,3), 'LineWidth', 1);
end
