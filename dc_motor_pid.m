%%
% DC Motor Speed Control with PID

r = 5;              % Motor winding resistance
L = 2;              % Winding inductance
Ke = 3;             % Back-EMF constant
Kt = 10;            % Current to torque constant
J = 5;              % Rotor inertia
B = 8;              % Mechanical friction

Dt = 0.001;         % Time step
Tsim = 10;          % Simulation duration
t = 0:Dt:Tsim;      % Time axis

% Initial motor values
current = zeros(size(t));
omega = zeros(size(t));

current(1) = 0;
omega(1) = 0;

V = 12;
T_load = 0;

% Motor response without PID
for i = 2:length(t)

    % Apply load torque after 5 seconds
    if t(i) < 5
        T_load = 0;
    else
        T_load = 2;
    end

    % Electrical equation
    Di = (V - current(i-1)*r - Ke*omega(i-1)) / L;

    % Mechanical equation
    Dw = (Kt*current(i-1) - B*omega(i-1) - T_load) / J;

    % Update motor values
    current(i) = current(i-1) + Di*Dt;
    omega(i) = omega(i-1) + Dw*Dt;
end

% Convert angular speed to RPM
RPM = omega * 60 / (2*pi);

% Plot motor response without PID
figure
plot(t,RPM)
xlabel("Time (s)")
ylabel("RPM")
title("DC Motor Speed Without PID")
grid on


% PID parameters
RPM_target = 15;

Kp = 10;
Ki = 1.1;
Kd = 0.5;

integral = 0;
Prev_error = 0;

% Initial PID motor values
current_pid = zeros(size(t));
omega_pid = zeros(size(t));
RPM_pid = zeros(size(t));

current_pid(1) = 0;
omega_pid(1) = 0;
RPM_pid(1) = 0;


% PID control loop
for i = 2:length(t)

    % Calculate speed error
    error = RPM_target - RPM_pid(i-1);

    % Integral term
    integral = integral + error*Dt;

    % Derivative term
    derivative = (error - Prev_error)/Dt;

    % Calculate control voltage
    V = Kp*error + Ki*integral + Kd*derivative;

    % Limit voltage between 0 and 12 V
    V = max(0,min(V,12));

    Prev_error = error;

    % Apply load after 5 seconds
    if t(i) < 5
        T_load = 0;
    else
        T_load = 2;
    end

    % Electrical equation
    Di_pid = (V - current_pid(i-1)*r - Ke*omega_pid(i-1)) / L;

    % Mechanical equation
    Dw_pid = (Kt*current_pid(i-1) - B*omega_pid(i-1) - T_load) / J;

    % Update motor values
    current_pid(i) = current_pid(i-1) + Di_pid*Dt;
    omega_pid(i) = omega_pid(i-1) + Dw_pid*Dt;

    % Convert speed to RPM
    RPM_pid(i) = omega_pid(i)*60/(2*pi);
end


% PID performance analysis

% Final speed and steady-state error
final_rpm = RPM_pid(end);
steady_state = RPM_target - final_rpm;

fprintf("Final RPM = %.3f RPM\n",final_rpm);
fprintf("Steady-State Error = %.3f RPM\n",steady_state);


% Maximum speed and overshoot
max_rpm = max(RPM_pid);
overshoot = ((max_rpm - RPM_target)/RPM_target)*100;

fprintf("Maximum RPM = %.2f RPM\n",max_rpm);
fprintf("Overshoot = %.2f %%\n",overshoot);


% Define 2 percent settling band
tolerance = 0.02;
limit = RPM_target*tolerance;

lower_limit = RPM_target - limit;
upper_limit = RPM_target + limit;

settling_time = 0;
settled = false;


% Find settling time
for i = 2:length(RPM_pid)

    if RPM_pid(i) >= lower_limit && RPM_pid(i) <= upper_limit

        settled = true;

        % Check if the response leaves the band later
        for j = i+1:length(RPM_pid)

            if RPM_pid(j) < lower_limit || RPM_pid(j) > upper_limit
                settled = false;
                break;
            end
        end

        if settled
            settling_time = t(i);
            break;
        end
    end
end

fprintf("Settling Time = %.3f s\n",settling_time);


% Calculate rise time
T10 = RPM_target*0.10;
T90 = RPM_target*0.90;

t10 = 0;
t90 = 0;

for i = 1:length(RPM_pid)

    if RPM_pid(i) >= T10 && t10 == 0
        t10 = t(i);
    end

    if RPM_pid(i) >= T90 && t90 == 0
        t90 = t(i);
    end
end

rise_time = t90 - t10;

fprintf("Rise Time = %.3f s\n",rise_time);


% Plot PID controlled response
figure
plot(t,RPM_pid)

xlabel("Time (s)")
ylabel("RPM")
title("DC Motor Speed Control with PID")

% Target speed
yline(RPM_target,'r--')

% 2 percent settling limits
yline(lower_limit,'k--')
yline(upper_limit,'k--')

grid on