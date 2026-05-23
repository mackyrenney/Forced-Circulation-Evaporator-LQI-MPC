%% output_data.m
fs = 14; 

tnon = out.tout; % or out.tout
L2_vals = out.L2_gen; 
X2_vals = out.X2_gen;
P2_vals = out.P2_gen;
F1_vals = out.F1_gen;
integrator_vals = out.integrator_gen;


figure(1); % Use a specific figure number
clf;     
set(gcf, 'Color', 'w');
sgtitle('LQI State Variables with UA2 +25%', 'FontSize', 16, 'FontWeight', 'bold')

subplot(3,1,1)
hold off; % Ensure we aren't drawing over old data
plot(tnon, L2_vals, 'b', 'LineWidth', 1.5)
ylabel('Level (m)', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([0, 2])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 

subplot(3,1,2)
hold off; % Explicitly turn hold off for the second plot
plot(tnon, X2_vals, 'b', 'LineWidth', 1.5)
ylabel('Composition (%)', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([10, 50])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 

subplot(3,1,3)
hold off; % Explicitly turn hold off for the third plot
plot(tnon, P2_vals, 'b', 'LineWidth', 1.5)
xlabel('Time (min)', 'FontSize', fs, 'FontWeight', 'bold') 
ylabel('Pressure (kPa)', 'FontSize', fs, 'FontWeight', 'bold')
ylim([50, 100])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 


% Extra Plots for step
subplot(4,1,1)
hold off; % Ensure we aren't drawing over old data
plot(tnon, F1_vals, 'r', 'LineWidth', 1.5)
ylabel('F1 flow (kg/min)', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([5, 15])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 

subplot(4,1,2)
hold off; % Ensure we aren't drawing over old data
plot(tnon, integrator_vals, 'r', 'LineWidth', 1.5)
ylabel('Integrating Gain', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([-700, 700])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 
