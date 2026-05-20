%% 1. Data Extraction (Squeezed for 1D vectors)
tnon      = squeeze(out.tout);
fs        = 12; 

% State Variables
L2_vals   = squeeze(out.L2_gen); 
X2_vals   = squeeze(out.X2_gen);
P2_vals   = squeeze(out.P2_gen);

% Flow/Pressure Variables
F2_vals   = squeeze(out.F2_gen);   
F3_vals   = squeeze(out.F3_gen);   
F200_vals = squeeze(out.F200_gen); 
P100_vals = squeeze(out.P100_gen); 

%% FIGURE 2: Process Flows & Pressures (Separated)
figure(2);
clf;
set(gcf, 'Color', 'w');
sgtitle('LQI Manipulated Variables with UA2 +25%', 'FontSize', 16, 'FontWeight', 'bold')

% Product Flow (F2)
subplot(4,1,1)
plot(tnon, F2_vals, 'b', 'LineWidth', 1.5)
ylabel('F2 (kg/min)', 'FontSize', fs, 'FontWeight', 'bold')
ylim([0, 5])
grid off;
set(gca, 'Color', 'w', 'LineWidth', 1.5, 'FontSize', 10, 'FontWeight', 'bold')

% Circulation Flow (F3)
subplot(4,1,2)
plot(tnon, F3_vals, 'b', 'LineWidth', 1.5)
ylabel('F3 (kg/min)', 'FontSize', fs, 'FontWeight', 'bold')
ylim([75, 85])
grid off;
set(gca, 'Color', 'w', 'LineWidth', 1.5, 'FontSize', 10, 'FontWeight', 'bold')

% Cooling Water (F200)
subplot(4,1,3)
plot(tnon, F200_vals, 'b', 'LineWidth', 1.5)
ylabel('F200 (kg/min)', 'FontSize', fs, 'FontWeight', 'bold')
ylim([50, 150])
grid off;
set(gca, 'Color', 'w', 'LineWidth', 1.5, 'FontSize', 10, 'FontWeight', 'bold')

% Steam Pressure (P100)
subplot(4,1,4)
plot(tnon, P100_vals, 'b', 'LineWidth', 1.5)
xlabel('Time (min)', 'FontSize', fs, 'FontWeight', 'bold') 
ylabel('P100 (kPa)', 'FontSize', fs, 'FontWeight', 'bold')
ylim([200, 350])
grid off;
set(gca, 'Color', 'w', 'LineWidth', 1.5, 'FontSize', 10, 'FontWeight', 'bold')