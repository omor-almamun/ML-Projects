clear;
clc;

% =========================================================================
% OBTAIN TRANSFER FUNCTIONS
% =========================================================================

disp('Obtaining transfer functions...');
[tf_thrust_to_Vt, tf_elevator_to_gamma] = get_transfer_functions();
tf_thrust_to_velocity = tf_thrust_to_Vt; % Rename for consistency

disp('Transfer functions obtained successfully:');
disp('Elevator to Flight Path Angle:');
disp(tf_elevator_to_gamma);
disp('Thrust to Velocity:');
disp(tf_thrust_to_velocity);

% =========================================================================
% PART 1: FLIGHT PATH ANGLE CONTROLLER DESIGN
% =========================================================================
disp('==========================================');
disp('FLIGHT PATH ANGLE CONTROLLER DESIGN');
disp('==========================================');

% Define actuator limits
elevator_min_rad = -0.4363; % -25 degrees in radians
elevator_max_rad = 0.4363;  % +25 degrees in radians

% PID Controller with alternative gains
Kp_gamma_alt = 0.05;        % Proportional gain
Kd_gamma_alt = 0.01;        % Derivative gain
Ki_gamma_alt = 0.005;       % Integral gain
N_alt = 50;                 % Filter coefficient

% Create PID controller
pid_num_alt = [Kd_gamma_alt, Kp_gamma_alt, Ki_gamma_alt];
pid_den_alt = [1/N_alt, 1, 0];
C_gamma_pid_alt = tf(pid_num_alt, pid_den_alt);
disp('Flight Path Angle PID Controller:');
disp(C_gamma_pid_alt);

% Closed-loop system
CL_gamma_pid_alt = feedback(C_gamma_pid_alt * tf_elevator_to_gamma, 1);
disp('Closed-Loop Transfer Function (PID):');
disp(CL_gamma_pid_alt);

% Stability check
poles_CL_gamma_pid_alt = pole(CL_gamma_pid_alt);
disp('Closed-Loop Poles (PID):');
disp(poles_CL_gamma_pid_alt);
is_stable_CL_gamma_pid_alt = all(real(poles_CL_gamma_pid_alt) < 0);
fprintf('Is the closed-loop system stable with PID? %s\n', mat2str(is_stable_CL_gamma_pid_alt));

% Lead-Lag Controller
lead_zero_alt = 1.0;        % Lead zero
lead_pole_alt = 10.0;       % Lead pole
lag_zero_alt = 0.1;         % Lag zero
lag_pole_alt = 0.01;        % Lag pole
K_leadlag_alt = 0.1;        % Gain

C_gamma_leadlag_alt = K_leadlag_alt * tf([1, lead_zero_alt], [1, lead_pole_alt]) * tf([1, lag_zero_alt], [1, lag_pole_alt]);
disp('Flight Path Angle Lead-Lag Controller:');
disp(C_gamma_leadlag_alt);

% Closed-loop system
CL_gamma_leadlag_alt = feedback(C_gamma_leadlag_alt * tf_elevator_to_gamma, 1);
disp('Closed-Loop Transfer Function (Lead-Lag):');
disp(CL_gamma_leadlag_alt);

% Stability check
poles_CL_gamma_leadlag_alt = pole(CL_gamma_leadlag_alt);
disp('Closed-Loop Poles (Lead-Lag):');
disp(poles_CL_gamma_leadlag_alt);
is_stable_CL_gamma_leadlag_alt = all(real(poles_CL_gamma_leadlag_alt) < 0);
fprintf('Is the closed-loop system stable with Lead-Lag? %s\n', mat2str(is_stable_CL_gamma_leadlag_alt));

% =========================================================================
% PART 2: VELOCITY CONTROLLER DESIGN
% =========================================================================
disp('==========================================');
disp('VELOCITY CONTROLLER DESIGN');
disp('==========================================');

% Define actuator limits in Newtons
thrust_min_N = 4448.22;     % 1000 lbf converted
thrust_max_N = 84514.18;    % 19000 lbf converted

% PID Controller with different gains
Kp_vel_alt = 150;           % Proportional gain
Kd_vel_alt = 50;            % Derivative gain
Ki_vel_alt = 5;             % Integral gain
N_vel_alt = 75;             % Filter coefficient

C_vel_pid_alt = tf([Kd_vel_alt, Kp_vel_alt, Ki_vel_alt], [1/N_vel_alt, 1, 0]);
disp('Velocity PID Controller:');
disp(C_vel_pid_alt);

% Closed-loop system
CL_vel_pid_alt = feedback(C_vel_pid_alt * tf_thrust_to_velocity, 1);
disp('Closed-Loop Transfer Function (PID):');
disp(CL_vel_pid_alt);

% Stability check
poles_CL_vel_pid_alt = pole(CL_vel_pid_alt);
disp('Closed-Loop Poles (PID):');
disp(poles_CL_vel_pid_alt);
is_stable_CL_vel_pid_alt = all(real(poles_CL_vel_pid_alt) < 0);
fprintf('Is the closed-loop system stable with PID? %s\n', mat2str(is_stable_CL_vel_pid_alt));

% Lead Controller
vel_lead_zero_alt = 0.8;    % Lead zero
vel_lead_pole_alt = 15.0;   % Lead pole
K_vel_lead_alt = 5000;      % Gain

C_vel_lead_alt = K_vel_lead_alt * tf([1, vel_lead_zero_alt], [1, vel_lead_pole_alt]);
disp('Velocity Lead Controller:');
disp(C_vel_lead_alt);

% Closed-loop system
CL_vel_lead_alt = feedback(C_vel_lead_alt * tf_thrust_to_velocity, 1);
disp('Closed-Loop Transfer Function (Lead):');
disp(CL_vel_lead_alt);

% Stability check
poles_CL_vel_lead_alt = pole(CL_vel_lead_alt);
disp('Closed-Loop Poles (Lead):');
disp(poles_CL_vel_lead_alt);
is_stable_CL_vel_lead_alt = all(real(poles_CL_vel_lead_alt) < 0);
fprintf('Is the closed-loop system stable with Lead? %s\n', mat2str(is_stable_CL_vel_lead_alt));

% =========================================================================
% PART 3: VISUALIZATION AND PERFORMANCE ANALYSIS
% =========================================================================
disp('==========================================');
disp('PERFORMANCE ANALYSIS');
disp('==========================================');

% Simulation parameters
t_alt = 0:0.02:50;          % 50s simulation time, 0.02s step
step_size_gamma_alt = 5 * (pi/180); % 5 degrees in radians
step_size_vel_alt = 10;     % 10 m/s

% Plot responses
figure(2);
subplot(2,1,1);
step(step_size_gamma_alt * CL_gamma_pid_alt, t_alt, 'g', step_size_gamma_alt * CL_gamma_leadlag_alt, t_alt, 'm--');
title('Flight Path Angle Response (5 deg step)');
ylabel('Flight Path Angle (rad)');
legend('PID', 'Lead-Lag');
grid on;

subplot(2,1,2);
step(step_size_vel_alt * CL_vel_pid_alt, t_alt, 'g', step_size_vel_alt * CL_vel_lead_alt, t_alt, 'm--');
title('Velocity Response (10 m/s step)');
ylabel('Velocity (m/s)');
xlabel('Time (s)');
legend('PID', 'Lead');
grid on;

% Control effort
figure(3);
subplot(2,1,1);
error_gamma_pid_alt = lsim(feedback(1, C_gamma_pid_alt * tf_elevator_to_gamma), step_size_gamma_alt * ones(size(t_alt)), t_alt);
u_gamma_pid_alt = lsim(C_gamma_pid_alt, error_gamma_pid_alt, t_alt);
error_gamma_leadlag_alt = lsim(feedback(1, C_gamma_leadlag_alt * tf_elevator_to_gamma), step_size_gamma_alt * ones(size(t_alt)), t_alt);
u_gamma_leadlag_alt = lsim(C_gamma_leadlag_alt, error_gamma_leadlag_alt, t_alt);
plot(t_alt, u_gamma_pid_alt, 'g', t_alt, u_gamma_leadlag_alt, 'm--', ...
     t_alt, elevator_max_rad * ones(size(t_alt)), 'k:', t_alt, elevator_min_rad * ones(size(t_alt)), 'k:');
title('Elevator Command (rad)');
ylabel('Elevator Angle (rad)');
legend('PID', 'Lead-Lag', 'Limits');
grid on;

subplot(2,1,2);
error_vel_pid_alt = lsim(feedback(1, C_vel_pid_alt * tf_thrust_to_velocity), step_size_vel_alt * ones(size(t_alt)), t_alt);
u_vel_pid_alt = lsim(C_vel_pid_alt, error_vel_pid_alt, t_alt);
error_vel_lead_alt = lsim(feedback(1, C_vel_lead_alt * tf_thrust_to_velocity), step_size_vel_alt * ones(size(t_alt)), t_alt);
u_vel_lead_alt = lsim(C_vel_lead_alt, error_vel_lead_alt, t_alt);
plot(t_alt, u_vel_pid_alt, 'g', t_alt, u_vel_lead_alt, 'm--', ...
     t_alt, thrust_max_N * ones(size(t_alt)), 'k:', t_alt, thrust_min_N * ones(size(t_alt)), 'k:');
title('Thrust Command (N)');
ylabel('Thrust (N)');
xlabel('Time (s)');
legend('PID', 'Lead', 'Limits');
grid on;

% Performance metrics
try
    info_gamma_pid_alt = stepinfo(step_size_gamma_alt * CL_gamma_pid_alt, t_alt);
    info_gamma_leadlag_alt = stepinfo(step_size_gamma_alt * CL_gamma_leadlag_alt, t_alt);
    fprintf('\nFlight Path Angle Controller Performance:\n');
    fprintf('PID - Rise time: %.2f s, Settling time: %.2f s, Overshoot: %.2f%%\n', ...
        info_gamma_pid_alt.RiseTime, info_gamma_pid_alt.SettlingTime, info_gamma_pid_alt.Overshoot);
    fprintf('Lead-Lag - Rise time: %.2f s, Settling time: %.2f s, Overshoot: %.2f%%\n', ...
        info_gamma_leadlag_alt.RiseTime, info_gamma_leadlag_alt.SettlingTime, info_gamma_leadlag_alt.Overshoot);

    info_vel_pid_alt = stepinfo(step_size_vel_alt * CL_vel_pid_alt, t_alt);
    info_vel_lead_alt = stepinfo(step_size_vel_alt * CL_vel_lead_alt, t_alt);
    fprintf('\nVelocity Controller Performance:\n');
    fprintf('PID - Rise time: %.2f s, Settling time: %.2f s, Overshoot: %.2f%%\n', ...
        info_vel_pid_alt.RiseTime, info_vel_pid_alt.SettlingTime, info_vel_pid_alt.Overshoot);
    fprintf('Lead - Rise time: %.2f s, Settling time: %.2f s, Overshoot: %.2f%%\n', ...
        info_vel_lead_alt.RiseTime, info_vel_lead_alt.SettlingTime, info_vel_lead_alt.Overshoot);
catch
    disp('Could not calculate all performance metrics due to simulation constraints');
end

% Check actuator usage
max_u_gamma_pid_alt = max(abs(u_gamma_pid_alt));
max_u_gamma_leadlag_alt = max(abs(u_gamma_leadlag_alt));
max_u_vel_pid_alt = max(abs(u_vel_pid_alt));
max_u_vel_lead_alt = max(abs(u_vel_lead_alt));

fprintf('\nActuator Usage:\n');
fprintf('Max elevator command (PID): %.2f rad (%.1f%% of range)\n', ...
    max_u_gamma_pid_alt, 100 * max_u_gamma_pid_alt / (elevator_max_rad - elevator_min_rad));
fprintf('Max elevator command (Lead-Lag): %.2f rad (%.1f%% of range)\n', ...
    max_u_gamma_leadlag_alt, 100 * max_u_gamma_leadlag_alt / (elevator_max_rad - elevator_min_rad));
fprintf('Max thrust command (PID): %.2f N (%.1f%% of range)\n', ...
    max_u_vel_pid_alt, 100 * (max_u_vel_pid_alt - thrust_min_N) / (thrust_max_N - thrust_min_N));
fprintf('Max thrust command (Lead): %.2f N (%.1f%% of range)\n', ...
    max_u_vel_lead_alt, 100 * (max_u_vel_lead_alt - thrust_min_N) / (thrust_max_N - thrust_min_N));