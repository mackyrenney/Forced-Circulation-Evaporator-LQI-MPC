% This code plots integrating states and a step in F1 input value for step testing

%% output_data.m
fs = 14; 

tnon = out.tout; 
F1_vals = out.F1_gen;
integrator_vals = out.integrator_gen;


figure(1); 
clf;       
set(gcf, 'Color', 'w');
sgtitle('F1 20% step load and Integrator Magnitude', 'FontSize', 16, 'FontWeight', 'bold')

subplot(4,1,1)
hold off; % Ensure we aren't drawing over old data
plot(tnon, F1_vals, 'r', 'LineWidth', 1.5)
ylabel('F1 (kg/min)', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([5, 15])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 

subplot(4,1,2)
hold off; % Ensure we aren't drawing over old data
plot(tnon, integrator_vals, 'r', 'LineWidth', 1.5)
ylabel('Integrator', 'FontSize', fs, 'FontWeight', 'bold') 
ylim([-4000, 4000])
set(gca, 'LineWidth', 2, 'FontSize', 12, 'FontWeight', 'bold') 
