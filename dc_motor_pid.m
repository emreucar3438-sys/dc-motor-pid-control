%%
r = 5; %% motor winding resistance
L = 2; %% winding inductance
Ke = 3; %% back-EMF constant
Kt = 10; %% current to torque constant
J = 5; %% rotor inertia
B = 8; %% mechanical friction
Dt = 0.001; %% time step
Tsim = 10; %% simulation duration
t = 0:Dt:Tsim; %% time axis

current_pid = zeros(size(t));
omega_pid = zeros(size(t));

current(1) = 0; %% initial current
omega(1) = 0; %% initial angular speed
V = 12; %% applied motor voltage
T_load = 0; %% initial load torque

%% Calculate motor response without PID
for i = 2:length(t)

    %% Apply load torque at 5 seconds
    if t(i) < 5
        T_load = 0;
    else
        T_load = 2;
    end

    %% Electrical motor equation
    Di = (V-(current(i-1)*r)-Ke*omega(i-1))/L;

    %% Mechanical motor equation
    Dw = ((Kt*current(i-1))-B*(omega(i-1))-T_load)/J;

    %% Update current and speed
    current(i) = current(i-1)+Di*Dt;
    omega(i) = omega(i-1)+Dw*Dt;
end

%% Convert angular speed to RPM
RPM = (omega*60)/(2*pi);

%% Plot motor speed without PID
figure
plot(t,RPM)
xlabel("Time")
ylabel("RPM")
grid on

%% PID controller parameters
RPM_target = 15; %% target motor speed
error = 0;
Prev_error = 0;

Kp = 10; %% proportional gain
Ki = 1.1; %% integral gain
Kd = 0.5; %% derivative gain

integral = 0;
derivative = 0;

%% Initialize PID motor variables
current_pid = zeros(size(t));
omega_pid = zeros(size(t));
RPM_pid = zeros(size(t));

current_pid(1) = 0;
omega_pid(1) = 0;
RPM_pid(1) = 0;

%% PID control loop
for i = 2:length(t)

    %% Calculate speed error
    error = RPM_target - RPM_pid(i-1);

    %% Calculate integral error
    integral = integral + error * Dt;

    %% Calculate derivative error
    derivative = (error - Prev_error) / Dt;

    %% Calculate PID output voltage
    V = Kp * error + Ki * integral + Kd * derivative;

    %% Limit motor voltage between 0 and 12 V
    V = max(0,min(V,12));

    Prev_error = error;

    %% Apply load torque at 5 seconds
    if t(i) < 5
        T_load = 0;
    else
        T_load = 2;
    end

    %% Electrical equation with PID control
    Di_pid = (V-(current_pid(i-1)*r)-Ke*omega_pid(i-1))/L;

    %% Mechanical equation with PID control
    Dw_pid = ((Kt*current_pid(i-1))-B*omega_pid(i-1)-T_load)/J;

    %% Update current and angular speed
    current_pid(i) = current_pid(i-1)+Di_pid*Dt;
    omega_pid(i) = omega_pid(i-1)+Dw_pid*Dt;

    %% Convert angular speed to RPM
    RPM_pid(i) = (omega_pid(i)*60)/(2*pi);
end

%% PID performance analysis

%% Calculate final speed and steady-state error
final_rpm = RPM_pid(end);
steady_state = RPM_target - final_rpm;

disp(final_rpm)
disp(steady_state)

%% Calculate maximum speed and overshoot
max_rpm = max(RPM_pid);
overshoot = ((max_rpm - RPM_target) / RPM_target) * 100;

fprintf("Maximum RPM = %.2f RPM\n", max_rpm);
fprintf("Overshoot = %.2f %%\n", overshoot);

%% Define the 2% settling band
tolerance = 0.02;
limit = RPM_target * tolerance;

lower_limit = RPM_target - limit;
upper_limit = RPM_target + limit;

settling_time = 0;
settled = false;

%% Find the time when the response stays inside the 2% band
for i = 2:length(RPM_pid)

    if RPM_pid(i) >= lower_limit && RPM_pid(i) <= upper_limit

        settled = true;

        %% Check if the response leaves the band later
        for j = i+1:length(RPM_pid)

            if RPM_pid(j) < lower_limit || RPM_pid(j) > upper_limit
                settled = false;
                break;
            end
        end

        %% Save the settling time
        if settled == true
            settling_time = t(i);
            break;
        end
    end
end

fprintf("Settling Time = %.3f s\n", settling_time);

%% Define 10% and 90% levels for rise time
T10 = RPM_target * 0.10;
T90 = RPM_target * 0.90;

t10 = 0;
t90 = 0;

%% Find the times when the response reaches 10% and 90%
for i = 1:length(RPM_pid)

    if RPM_pid(i) >= T10 && t10 == 0
        t10 = t(i);
    end

    if RPM_pid(i) >= T90 && t90 == 0
        t90 = t(i);
    end
end

%% Calculate rise time
rise_time = t90 - t10;

fprintf("Rise Time = %.3f s\n", rise_time);

%% Plot PID controlled motor response
figure
plot(t,RPM_pid)

xlabel("Time")
ylabel("RPM")
title("DC Motor Speed Control with PID")

%% Show target speed and 2% settling limits
yline(RPM_target, 'r--')
yline(lower_limit, 'k--')
yline(upper_limit, 'k--')

grid on