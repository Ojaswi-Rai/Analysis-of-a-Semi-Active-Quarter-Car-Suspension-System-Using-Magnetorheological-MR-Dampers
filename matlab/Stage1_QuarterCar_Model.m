
%% STAGE 1: TWO-DOF QUARTER CAR MODEL
% MR Damper-Based Semi-Active Suspension Project
% Initial passive suspension model

clear;
clc;
close all;

%% 1. SYSTEM PARAMETERS

p.ms = 300;       % Sprung mass (kg)
p.mu = 40;        % Unsprung mass (kg)

p.ks = 18000;     % Suspension stiffness (N/m)
p.kt = 180000;    % Tyre stiffness (N/m)

p.c = 1000;       % Fixed damping coefficient (Ns/m)

%% 2. SIMULATION SETTINGS

t_start = 0;
t_end = 5;

tspan = [t_start t_end];

% Initial conditions:
% [Body displacement; Body velocity;
%  Wheel displacement; Wheel velocity]

x0 = [0; 0; 0; 0];

%% 3. SOLVE EQUATIONS OF MOTION

options = odeset('RelTol',1e-7, ...
                 'AbsTol',1e-9, ...
                 'MaxStep',0.002);

[t, x] = ode45(@(t,x) quarterCarODE(t,x,p), ...
               tspan, x0, options);

%% 4. EXTRACT SYSTEM RESPONSES

z_s = x(:,1);       % Sprung mass displacement
v_s = x(:,2);       % Sprung mass velocity

z_u = x(:,3);       % Unsprung mass displacement
v_u = x(:,4);       % Unsprung mass velocity

% Road displacement

z_r = arrayfun(@roadProfile,t);

%% 5. CALCULATE ACCELERATIONS

a_s = (-p.ks*(z_s-z_u) ...
       -p.c*(v_s-v_u))/p.ms;

a_u = (p.ks*(z_s-z_u) ...
       +p.c*(v_s-v_u) ...
       -p.kt*(z_u-z_r))/p.mu;

%% 6. CALCULATE PERFORMANCE VARIABLES

suspension_travel = z_s-z_u;

tyre_deflection = z_u-z_r;

%% 7. PLOT RESULTS

figure('Name','Quarter Car Model','Color','w');

subplot(3,2,1);
plot(t,z_r,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Road displacement (m)');
title('Road Input');

subplot(3,2,2);
plot(t,z_s,'LineWidth',1.5);
hold on;
plot(t,z_u,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Displacement (m)');
title('Mass Displacements');
legend('Sprung Mass','Unsprung Mass');

subplot(3,2,3);
plot(t,a_s,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Sprung Mass Acceleration');

subplot(3,2,4);
plot(t,suspension_travel,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Travel (m)');
title('Suspension Travel');

subplot(3,2,5);
plot(t,tyre_deflection,'LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Deflection (m)');
title('Tyre Deflection');

sgtitle('Passive Quarter-Car Suspension Response');

%% 8. DISPLAY BASIC RESULTS

fprintf('\nQUARTER CAR SIMULATION COMPLETED\n');

fprintf('Maximum body displacement: %.5f m\n', ...
        max(abs(z_s)));

fprintf('Maximum body acceleration: %.5f m/s^2\n', ...
        max(abs(a_s)));

fprintf('Maximum suspension travel: %.5f m\n', ...
        max(abs(suspension_travel)));

fprintf('Maximum tyre deflection: %.5f m\n', ...
        max(abs(tyre_deflection)));

%% LOCAL FUNCTIONS

function dx = quarterCarODE(t,x,p)

    % Extract states

    z_s = x(1);
    v_s = x(2);

    z_u = x(3);
    v_u = x(4);

    % Road excitation

    z_r = roadProfile(t);

    % Spring and damper forces

    F_s = p.ks*(z_s-z_u);

    F_d = p.c*(v_s-v_u);

    % Equations of motion

    a_s = (-F_s-F_d)/p.ms;

    a_u = (F_s+F_d-p.kt*(z_u-z_r))/p.mu;

    % State derivatives

    dx = [v_s;
          a_s;
          v_u;
          a_u];

end

function z_r = roadProfile(t)

    bump_height = 0.05;
    bump_start = 1;
    bump_duration = 0.2;

    if t >= bump_start && ...
       t <= bump_start+bump_duration

        z_r = bump_height*sin( ...
            pi*(t-bump_start)/bump_duration);

    else

        z_r = 0;

    end

end
