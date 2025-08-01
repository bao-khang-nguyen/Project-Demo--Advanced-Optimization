% === MANUAL PID Parameters ===
Kp_manual = 0.01;
Ki_manual = 5;
Kd_manual = 0;

% === GA-Optimized PID Parameters ===
Kp_ga = 0.0052;
Ki_ga = 5.9417;
Kd_ga = 0.0;

% === PSO-Optimized PID Parameters ===
Kp_pso = 0.0050;
Ki_pso = 5.9987;
Kd_pso = 0.0;

% === Simulate MANUAL PID ===
assignin('base','Kp', Kp_manual);
assignin('base','Ki', Ki_manual);
assignin('base','Kd', Kd_manual);
simManual = sim('buck_model2');

% === Simulate GA PID ===
assignin('base','Kp', Kp_ga);
assignin('base','Ki', Ki_ga);
assignin('base','Kd', Kd_ga);
simGA = sim('buck_model2');

% === Simulate PSO PID ===
assignin('base','Kp', Kp_pso);
assignin('base','Ki', Ki_pso);
assignin('base','Kd', Kd_pso);
simPSO = sim('buck_model2');

% === Extract data ===
t_manual = simManual.logsout.getElement('Vout').Values.Time;
v_manual = simManual.logsout.getElement('Vout').Values.Data;

t_ga = simGA.logsout.getElement('Vout').Values.Time;
v_ga = simGA.logsout.getElement('Vout').Values.Data;

t_pso = simPSO.logsout.getElement('Vout').Values.Time;
v_pso = simPSO.logsout.getElement('Vout').Values.Data;

% === Plot all three responses ===
figure('Name','Manual vs GA vs PSO PID Comparison','NumberTitle','off');
plot(t_manual, v_manual, 'r--', 'LineWidth', 1.5); hold on;
plot(t_ga, v_ga, 'b-', 'LineWidth', 2);
plot(t_pso, v_pso, 'g-.', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('V_{out} (V)');
title('PID Response Comparison: Manual vs GA vs PSO');
legend(...
    sprintf('Manual PID (Kp=%.4f, Ki=%.1f, Kd=%.1f)', Kp_manual, Ki_manual, Kd_manual), ...
    sprintf('GA PID (Kp=%.4f, Ki=%.1f, Kd=%.1f)', Kp_ga, Ki_ga, Kd_ga), ...
    sprintf('PSO PID (Kp=%.4f, Ki=%.4f, Kd=%.1f)', Kp_pso, Ki_pso, Kd_pso) ...
);

% === Optional: Save Figure ===
saveas(gcf, 'fig_manual_ga_pso_comparison.png');
