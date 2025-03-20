function [tf_thrust_to_Vt, tf_elevator_to_gamma] = get_transfer_functions()
    % Load conversion constants and parameters
    conversion; % Defines ft2m, d2r, etc.
    Param = load_F16_params(); % F16 parameters

    % Set initial conditions for trimming
    h0 = -10000 * ft2m; % Altitude
    Vt0 = 300 * ft2m;   % Initial velocity
    IC.inertial_position = [0, 0, h0];
    IC.body_velocity = [Vt0, 0, 0];
    IC.euler_angles = [0, 0, 0] * d2r;
    IC.omega = [0, 0, 0];
    Tend = 2.0;

    % Assign variables to base workspace for Simulink
    assignin('base', 'IC', IC);
    assignin('base', 'Param', Param);
    assignin('base', 'Tend', Tend);

    % Verify variables are in base workspace
    disp('Verifying base workspace variables:');
    evalin('base', 'disp(exist(''IC'', ''var''))'); % Should print 1
    evalin('base', 'disp(exist(''Param'', ''var''))'); % Should print 1
    evalin('base', 'disp(exist(''Tend'', ''var''))'); % Should print 1

    % Trim and linearize the F16 model
    try
        TrimF16; % Sets operating point 'op'
        linsys = linearize('F16', op);
    catch ME
        disp('Error during trimming/linearization:');
        disp(ME.message);
        rethrow(ME);
    end

    % Clean up base workspace
    evalin('base', 'clear IC Param Tend');

    % Define longitudinal state indices
    long_states = [5, 7, 9, 11]; % theta, Vt, alpha, q
    input_indices = [1, 2];      % Thrust, Elevator

    % Extract longitudinal state-space model
    A_long = linsys.a(long_states, long_states);
    B_long = linsys.b(long_states, input_indices);
    C_long = eye(4);             % Output all states
    D_long = zeros(4, 2);

    % Create state-space system
    sys_long = ss(A_long, B_long, C_long, D_long);

    % Define gamma as theta - alpha
    gamma_C = [1, 0, -1, 0]; % gamma = theta - alpha
    gamma_D = [0, 0];
    sys_long_with_gamma = ss(A_long, B_long, [C_long; gamma_C], [D_long; gamma_D]);

    % Extract transfer functions
    tf_thrust_to_Vt = tf(sys_long(2, 1));         % Vt (state 2) from Thrust (input 1)
    tf_elevator_to_gamma = tf(sys_long_with_gamma(5, 2)); % gamma (output 5) from Elevator (input 2)
end