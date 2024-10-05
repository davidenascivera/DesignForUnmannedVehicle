% Import the CSV file data and plot columns 2-4 and 5-7 in 3D space

% Import the data
filename = 'ekf_logs/ekf_estimates_20241005_114715.csv'; % Replace with your CSV file name
data = readmatrix(filename);

% Extract position and velocity columns
position = data(:, 2:4);
velocity = data(:, 5:7);

% Create a 3D scatter plot
figure;
scatter3(position(:, 1), position(:, 2), position(:, 3), 'r', 'DisplayName', 'Position');
hold on;
scatter3(velocity(:, 1), velocity(:, 2), velocity(:, 3), 'b', 'DisplayName', 'Velocity');

% Set labels and legend
xlabel('X Axis');
ylabel('Y Axis');
zlabel('Z Axis');
legend;

grid on;
hold off;