function [state, options, optchanged] = gaLogger(options, state, flag)
% gaLogger - Custom output function for GA PID tuning
% Logs best candidate per generation with detailed metrics.

    persistent bestHistory bestCosts;
    optchanged = false;  % Default: do not change GA behavior

    switch flag
        case 'init'
            % Initialize persistent logs
            bestHistory = [];
            bestCosts   = [];

        case 'iter'
            % Identify current best individual
            [bestCost, bestIdx] = min(state.Score);
            K = state.Population(bestIdx, :);  % [Kp, Ki, Kd]

            % Attempt to fetch last evaluated metrics from the base workspace
            try
                IAE        = evalin('base', 'lastIAE');
                ISE        = evalin('base', 'lastISE');
                ITSE       = evalin('base', 'lastITSE');
                ISTSE      = evalin('base', 'lastISTSE');
                rise       = evalin('base', 'lastRisePenalty');
                over       = evalin('base', 'lastOvershootPenalty');
                jerk       = evalin('base', 'lastJerkPenalty');
                settle     = evalin('base', 'lastSettlingPenalty');
                cost       = evalin('base', 'lastCost');
                os         = evalin('base', 'lastOvershoot');
                undershoot = evalin('base', 'lastUndershootPenalty');
                currK      = evalin('base', 'currentPID');

                % Print generation log
                fprintf('\n[Generation %d] Current Best Candidate:\n', state.Generation);
                fprintf('   Kp = %.4f, Ki = %.4f, Kd = %.4f\n', K(1), K(2), K(3));
                fprintf('   Composite Cost J = %.4f\n', cost);
                fprintf('   Overshoot = %.2f%% | Rise Time Penalty = %.2fs\n', os, rise);
                fprintf('   Penalties => Overshoot: %.2f | Undershoot: %.2f | Jerk: %.2f | Settling: %.2f\n', ...
                        over, undershoot, jerk, settle);
                fprintf('   Metrics => IAE: %.2f | ISE: %.2f | ITSE: %.2f | ISTSE: %.2f\n', ...
                        IAE, ISE, ITSE, ISTSE);
                fprintf('   [Last Evaluated Candidate] Kp=%.4f, Ki=%.4f, Kd=%.4f\n', currK(1), currK(2), currK(3));

            catch
                fprintf('[Warning] Metrics not yet available for generation %d.\n', state.Generation);
            end

            % Store cost and best candidate for later analysis
            bestCosts(end+1) = bestCost;

            bestHistory(end+1).Generation = state.Generation;
            bestHistory(end).Kp = K(1);
            bestHistory(end).Ki = K(2);
            bestHistory(end).Kd = K(3);
            bestHistory(end).Cost = bestCost;

        case 'done'
            % Assign logged results to base workspace for post-processing
            assignin('base', 'bestHistory', bestHistory);
            assignin('base', 'bestCosts', bestCosts);

            fprintf('\nGA optimization complete. Logged %d generations.\n', numel(bestCosts));
    end
end
