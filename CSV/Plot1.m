% Read in the CSV file
data = readmatrix('Initial2D.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);

% Plot the values
figure;
plot(x, y);
xlabel('x');
ylabel('y');
grid on;
hold on;

data = readmatrix('result2DBatch.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);

% Plot the values
plot(x, y);
xlabel('x');
ylabel('y');
grid on;
% Add the legend
axis equal
legend('Unoptimized Trajectory', 'optimized Trajectory');




% Read in the CSV file
data = readmatrix('Initial2D.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);

% Plot the values
figure;
plot(x, y);
xlabel('x');
ylabel('y');
grid on;
hold on;

data = readmatrix('result2DIncremental.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);

% Plot the values
plot(x, y);
xlabel('x');
ylabel('y');
grid on;
% Add the legend
axis equal;
legend('Unoptimized Trajectory', 'optimized Trajectory');


% Read in the CSV file
data = readmatrix('initial3D.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Plot the values
figure;
plot3(x, y, z);
xlabel('x');
ylabel('y');
zlabel('z')
grid on;
hold on;

% Read in the CSV file
data = readmatrix('result3DBatch.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Plot the values
plot3(x, y, z);
xlabel('x');
ylabel('y');
zlabel('z')
grid on;
hold on;

% Add the legend
legend('Unoptimized Trajectory', 'optimized Trajectory');



% Read in the CSV file
data = readmatrix('initial3D.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Plot the values
figure;
plot3(x, y, z);
xlabel('x');
ylabel('y');
zlabel('z')
grid on;
hold on;

% Read in the CSV file
data = readmatrix('result3DIncremental.csv', 'Delimiter', ',');

% Extract the x, y, and z columns
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Plot the values
plot3(x, y, z);
xlabel('x');
ylabel('y');
zlabel('z')
grid on;
hold on;

% Add the legend
legend('Unoptimized Trajectory', 'optimized Trajectory');
