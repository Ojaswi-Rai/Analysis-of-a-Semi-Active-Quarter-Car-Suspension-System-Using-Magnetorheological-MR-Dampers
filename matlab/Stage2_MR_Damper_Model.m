
%% STAGE 2: MR DAMPER MATHEMATICAL MODEL
% Bingham Model
% MR Damper-Based Semi-Active Suspension Project

clear;
clc;
close all;

%% 1. MR DAMPER PARAMETERS

p.c_min = 800;       % Minimum damping (Ns/m)
p.c_max = 5000;      % Maximum damping (Ns/m)

p.I_max = 2;         % Maximum current (A)

p.Fy_max = 1000;     % Maximum yield force (N)

p.v_eps = 0.001;     % Velocity smoothing parameter (m/s)

%% 2. DEFINE CURRENT AND VELOCITY RANGES

I_values = [0 0.5 1.0 1.5 2.0];

velocity = linspace(-0.5,0.5,1001);

%% 3. CALCULATE FORCE-VELOCITY CHARACTERISTICS

figure('Name','MR Damper Characteristics','Color','w');

subplot(2,2,[1 2]);

hold on;

for i = 1:length(I_values)

    I = I_values(i);

    Fd = mrDamperForce(velocity,I,p);

    plot(velocity,Fd,'LineWidth',1.5);

end

grid on;

xlabel('Relative Velocity (m/s)');
ylabel('Damper Force (N)');

title('MR Damper Force-Velocity Characteristics');

legend('0 A','0.5 A','1 A','1.5 A','2 A', ...
       'Location','best');

%% 4. DAMPING COEFFICIENT VS CURRENT

current_range = linspace(0,p.I_max,100);

c_values = p.c_min + ...
    (p.c_max-p.c_min).*current_range/p.I_max;

subplot(2,2,3);

plot(current_range,c_values,'LineWidth',1.5);

grid on;

xlabel('Applied Current (A)');
ylabel('Damping Coefficient (Ns/m)');

title('Damping Coefficient vs Current');

%% 5. YIELD FORCE VS CURRENT

Fy_values = p.Fy_max.*current_range/p.I_max;

subplot(2,2,4);

plot(current_range,Fy_values,'LineWidth',1.5);

grid on;

xlabel('Applied Current (A)');
ylabel('Yield Force (N)');

title('Yield Force vs Current');

sgtitle('Bingham MR Damper Model');

%% 6. CHECK FORCE AT DIFFERENT CURRENTS

fprintf('\nMR DAMPER MODEL RESULTS\n');

fprintf('\nCurrent | Damping | Yield Force\n');
fprintf('   (A)  | (Ns/m)  |    (N)\n');

for i = 1:length(I_values)

    I = I_values(i);

    c0 = p.c_min + ...
        (p.c_max-p.c_min)*I/p.I_max;

    Fy = p.Fy_max*I/p.I_max;

    fprintf(' %5.2f  | %7.2f | %10.2f\n', ...
            I,c0,Fy);

end

%% 7. CHECK PHYSICAL DISSIPATION CONDITION

fprintf('\nCHECKING DAMPER DISSIPATION:\n');

for i = 1:length(I_values)

    Fd = mrDamperForce(velocity,I_values(i),p);

    power = Fd.*velocity;

    if min(power) >= -1e-10

        fprintf('Current %.2f A: PASS\n',I_values(i));

    else

        fprintf('Current %.2f A: FAIL\n',I_values(i));

    end

end

%% 8. TEST FORCE AT A FIXED VELOCITY

test_velocity = 0.1;

fprintf('\nFORCE AT RELATIVE VELOCITY = %.2f m/s\n', ...
        test_velocity);

for i = 1:length(I_values)

    Fd = mrDamperForce(test_velocity,I_values(i),p);

    fprintf('Current %.2f A: Force = %.2f N\n', ...
            I_values(i),Fd);

end

%% LOCAL FUNCTION

function Fd = mrDamperForce(v_rel,I,p)

    % Limit current to physically allowed range

    I = max(0,min(I,p.I_max));

    % Current-dependent damping coefficient

    c0 = p.c_min + ...
         (p.c_max-p.c_min)*I/p.I_max;

    % Current-dependent yield force

    Fy = p.Fy_max*I/p.I_max;

    % Smooth approximation of sign function

    smooth_sign = tanh(v_rel/p.v_eps);

    % Bingham model

    Fd = c0.*v_rel + Fy.*smooth_sign;

end
