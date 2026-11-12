
%% STAGE 3: INTEGRATED MR DAMPER QUARTER-CAR MODEL
% MR Damper-Based Semi-Active Suspension Project
% Fixed-current simulation and passive comparison

clear;
clc;
close all;

%% 1. QUARTER CAR PARAMETERS

p.ms = 300;          % Sprung mass (kg)
p.mu = 40;           % Unsprung mass (kg)

p.ks = 18000;        % Suspension stiffness (N/m)
p.kt = 180000;       % Tyre stiffness (N/m)

%% 2. MR DAMPER PARAMETERS

p.c_min = 800;       % Minimum damping (Ns/m)
p.c_max = 5000;      % Maximum damping (Ns/m)

p.I_max = 2;         % Maximum current (A)
p.Fy_max = 1000;     % Maximum yield force (N)

p.v_eps = 0.001;     % Smoothing velocity (m/s)

%% 3. PASSIVE SUSPENSION PARAMETERS

p.c_passive = 1000;  % Conventional damper (Ns/m)

%% 4. SIMULATION SETTINGS

tspan = [0 5];

x0 = [0; 0; 0; 0];

options = odeset('RelTol',1e-7, ...
                 'AbsTol',1e-9, ...
                 'MaxStep',0.002);

%% 5. DEFINE SIMULATION CASES

currents = [0 0.5 1 1.5 2];

number_of_cases = length(currents)+1;

results = struct();

%% 6. RUN PASSIVE AND MR DAMPER SIMULATIONS

for j = 1:number_of_cases

    if j == 1

        % Conventional passive suspension

        mode = 0;
        I = 0;

        case_name = 'Passive';

    else

        % MR damper at fixed current

        mode = 1;
        I = currents(j-1);

        case_name = sprintf('MR %.1f A',I);

    end

    % Solve quarter-car equations

    [t,x] = ode45( ...
        @(t,x) integratedQuarterCar(t,x,p,mode,I), ...
        tspan,x0,options);

    % Extract states

    zs = x(:,1);
    vs = x(:,2);

    zu = x(:,3);
    vu = x(:,4);

    % Road displacement

    zr = arrayfun(@roadProfile,t);

    % Calculate accelerations

    as = zeros(size(t));
    au = zeros(size(t));

    Fd = zeros(size(t));

    for k = 1:length(t)

        [as(k),au(k),Fd(k)] = ...
            calculateAcceleration(zs(k),vs(k), ...
                                  zu(k),vu(k),zr(k), ...
                                  p,mode,I);

    end

    % Calculate performance variables

    suspension_travel = zs-zu;

    tyre_deflection = zu-zr;

    % Store results

    results(j).name = case_name;
    results(j).current = I;

    results(j).t = t;
    results(j).zs = zs;
    results(j).zu = zu;

    results(j).as = as;
    results(j).au = au;

    results(j).zr = zr;
    results(j).Fd = Fd;

    results(j).suspension_travel = suspension_travel;
    results(j).tyre_deflection = tyre_deflection;

    % Performance metrics

    results(j).max_body_displacement = max(abs(zs));

    results(j).max_body_acceleration = max(abs(as));

    results(j).rms_body_acceleration = ...
    sqrt(trapz(t,as.^2)/(t(end)-t(1)));

    results(j).max_suspension_travel = ...
        max(abs(suspension_travel));

    results(j).max_tyre_deflection = ...
        max(abs(tyre_deflection));

    results(j).max_damper_force = max(abs(Fd));

end

%% 7. DISPLAY PERFORMANCE COMPARISON

fprintf('\nINTEGRATED MR SUSPENSION RESULTS\n');

fprintf('\n%-12s %12s %12s %12s %12s\n', ...
    'Case','Body Disp','Peak Acc','RMS Acc','Susp Travel');

fprintf('%s\n',repmat('-',1,67));

for j = 1:number_of_cases

    fprintf('%-12s %9.3f mm %9.3f %9.3f %9.3f mm\n', ...
        results(j).name, ...
        results(j).max_body_displacement*1000, ...
        results(j).max_body_acceleration, ...
        results(j).rms_body_acceleration, ...
        results(j).max_suspension_travel*1000);

end

%% 8. PLOT BODY DISPLACEMENT

figure('Name','Body Displacement Comparison','Color','w');

hold on;

for j = 1:number_of_cases

    plot(results(j).t,results(j).zs*1000, ...
        'LineWidth',1.4);

end

grid on;

xlabel('Time (s)');
ylabel('Body Displacement (mm)');

title('Sprung Mass Displacement Comparison');

legend({results.name},'Location','best');

%% 9. PLOT BODY ACCELERATION

figure('Name','Body Acceleration Comparison','Color','w');

hold on;

for j = 1:number_of_cases

    plot(results(j).t,results(j).as, ...
        'LineWidth',1.4);

end

grid on;

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

title('Sprung Mass Acceleration Comparison');

legend({results.name},'Location','best');

%% 10. PLOT SUSPENSION TRAVEL

figure('Name','Suspension Travel Comparison','Color','w');

hold on;

for j = 1:number_of_cases

    plot(results(j).t, ...
         results(j).suspension_travel*1000, ...
         'LineWidth',1.4);

end

grid on;

xlabel('Time (s)');
ylabel('Suspension Travel (mm)');

title('Suspension Travel Comparison');

legend({results.name},'Location','best');

%% 11. PLOT TYRE DEFLECTION

figure('Name','Tyre Deflection Comparison','Color','w');

hold on;

for j = 1:number_of_cases

    plot(results(j).t, ...
         results(j).tyre_deflection*1000, ...
         'LineWidth',1.4);

end

grid on;

xlabel('Time (s)');
ylabel('Tyre Deflection (mm)');

title('Tyre Deflection Comparison');

legend({results.name},'Location','best');

%% 12. PLOT MR DAMPER FORCE

figure('Name','MR Damper Force','Color','w');

hold on;

for j = 2:number_of_cases

    plot(results(j).t,results(j).Fd, ...
        'LineWidth',1.3);

end

grid on;

xlabel('Time (s)');
ylabel('Damper Force (N)');

title('MR Damper Force at Different Currents');

legend({results(2:end).name},'Location','best');

%% 13. DISPLAY ADDITIONAL METRICS

fprintf('\nADDITIONAL PERFORMANCE RESULTS\n');

for j = 1:number_of_cases

    fprintf('\n%s\n',results(j).name);

    fprintf('Maximum tyre deflection: %.3f mm\n', ...
        results(j).max_tyre_deflection*1000);

    fprintf('Maximum damper force: %.2f N\n', ...
        results(j).max_damper_force);

    fprintf('RMS body acceleration: %.5f m/s^2\n', ...
        results(j).rms_body_acceleration);

end

fprintf('\nSTAGE 3 SIMULATION COMPLETED\n');

%% LOCAL FUNCTION 1: QUARTER-CAR EQUATIONS

function dx = integratedQuarterCar(t,x,p,mode,I)

    zs = x(1);
    vs = x(2);

    zu = x(3);
    vu = x(4);

    zr = roadProfile(t);

    [as,au,~] = calculateAcceleration( ...
        zs,vs,zu,vu,zr,p,mode,I);

    dx = [vs; as; vu; au];

end

%% LOCAL FUNCTION 2: FORCE AND ACCELERATION

function [as,au,Fd] = calculateAcceleration( ...
    zs,vs,zu,vu,zr,p,mode,I)

    relative_velocity = vs-vu;

    spring_force = p.ks*(zs-zu);

    tyre_force = p.kt*(zu-zr);

    if mode == 0

        % Conventional passive damper

        Fd = p.c_passive*relative_velocity;

    else

        % Current-dependent Bingham MR damper

        I = max(0,min(I,p.I_max));

        c0 = p.c_min + ...
            (p.c_max-p.c_min)*I/p.I_max;

        Fy = p.Fy_max*I/p.I_max;

        Fd = c0*relative_velocity + ...
             Fy*tanh(relative_velocity/p.v_eps);

    end

    % Equations of motion

    as = (-spring_force-Fd)/p.ms;

    au = (spring_force+Fd-tyre_force)/p.mu;

end

%% LOCAL FUNCTION 3: ROAD PROFILE

function zr = roadProfile(t)

    bump_height = 0.05;

    bump_start = 1;

    bump_duration = 0.2;

    if t >= bump_start && ...
       t <= bump_start+bump_duration

        zr = bump_height*sin( ...
            pi*(t-bump_start)/bump_duration);

    else

        zr = 0;

    end

end
