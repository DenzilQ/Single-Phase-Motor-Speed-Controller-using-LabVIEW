% Define the Plant Transfer Function
numerator_plant = 2604;
denominator_plant = [1, 0.7092, 0.36];
G = tf(numerator_plant, denominator_plant);

% Define the PID Controller Transfer Function with initial gains
Kp = 0.04; % Reduced proportional gain
Ki = 0.015;
Kd = 0.0025;
numerator_PID = [Kd, Kp, Ki];
denominator_PID = [1, 0];
C = tf(numerator_PID, denominator_PID);

% Combine the Plant and PID Controller for the Open-Loop Transfer Function
L = series(C, G);

% Compute the Closed-Loop Transfer Function
T = feedback(L, 1);

% Plot the Step Response of the Closed-Loop System
% stepinfo(T)

% Plot the Step Response of the Closed-Loop System
% step(T);
%title('Closed-Loop Step Response');
%grid on;



