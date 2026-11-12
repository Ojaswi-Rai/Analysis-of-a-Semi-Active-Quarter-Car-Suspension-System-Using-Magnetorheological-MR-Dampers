
%% ============================================================
% STAGE 4: FINAL SKYHOOK AND PID CONTROL SIMULATION
% MR DAMPER QUARTER-CAR SUSPENSION MODEL
%
% Simulation method: Fixed-step 4th Order Runge-Kutta (RK4)
%
% Simulation cases:
% 1. Passive suspension
% 2. MR Damper - 0 A
% 3. MR Damper - 0.5 A
% 4. MR Damper - 1 A
% 5. MR Damper - 1.5 A
% 6. MR Damper - 2 A
% 7. Skyhook controlled MR damper
% 8. PID controlled MR damper
%
% Additional diagnostic graphs:
% 7. Relative damper velocity at 2 A
%
% MATLAB R2016b or newer
%% ============================================================

clear;
clc;
close all;

%% 1. QUARTER-CAR PARAMETERS

p.ms = 300;              % Sprung mass (kg)
p.mu = 40;               % Unsprung mass (kg)

p.ks = 18000;            % Suspension stiffness (N/m)
p.kt = 180000;           % Tyre stiffness (N/m)

%% 2. MR DAMPER PARAMETERS

p.c0 = 800;              % Base damping coefficient (Ns/m)
p.cSlope = 2100;         % Current-dependent damping (Ns/m/A)

p.FySlope = 500;         % Yield force slope (N/A)

p.Imin = 0;
p.Imax = 2;

p.Istep = 0.02;          % Current mapping resolution (A)

p.vEps = 0.005;          % Smooth yield transition (m/s)

p.tauI = 0.03;           % Current response time constant (s)

p.maxSlew = 100;         % Maximum current change rate (A/s)

%% 3. PASSIVE DAMPER

% Separate passive suspension baseline.
% MR damper at 0 A has c = 800 Ns/m.
% Passive suspension has c = 1000 Ns/m.

p.cPassive = 1000;

%% 4. SIMULATION PARAMETERS

dt = 0.001;

T = 5;

t = 0:dt:T;

N = length(t);

% Half-sine road bump

p.bumpHeight = 0.05;
p.bumpStart = 1.0;
p.bumpDuration = 0.2;

%% 5. CONTROLLER PARAMETERS

% SKYHOOK CONTROLLER

p.Csky = 2500;

p.skyHyst = 0.002;

p.vDeadband = 0.002;

% PID CONTROLLER

p.Kp = 18000;
p.Ki = 1000;
p.Kd = 3500;

p.ItermLimit = 0.15;

% FIXED CURRENT CASES

fixedCurrents = [0 0.5 1 1.5 2];

%% 6. DEFINE SIMULATION CASES

caseNames = { ...
    'Passive', ...
    'MR 0 A', ...
    'MR 0.5 A', ...
    'MR 1 A', ...
    'MR 1.5 A', ...
    'MR 2 A', ...
    'Skyhook', ...
    'PID'};

numCases = length(caseNames);

fprintf('\n============================================\n');
fprintf('STAGE 4: FINAL CONTROL SIMULATION\n');
fprintf('============================================\n');

%% 7. RUN ALL SIMULATIONS

for k = 1:numCases

    fprintf('Running %s...\n',caseNames{k});

    if k == 1

        mode = 'passive';
        fixedI = 0;

    elseif k <= 6

        mode = 'fixed';
        fixedI = fixedCurrents(k-1);

    elseif k == 7

        mode = 'skyhook';
        fixedI = 0;

    else

        mode = 'pid';
        fixedI = 0;

    end

    results(k) = simulateSuspension( ...
        t,dt,p,mode,fixedI);

end

%% 8. CALCULATE PERFORMANCE METRICS

fprintf('\n');
fprintf('============================================\n');
fprintf('STAGE 4 PERFORMANCE COMPARISON\n');
fprintf('============================================\n');

fprintf('%-12s %12s %12s %12s %12s\n', ...
    'System','Disp(mm)','PeakAcc','RMSAcc','Travel(mm)');

fprintf('%s\n',repmat('-',1,66));

% Columns:
% 1 = Body displacement (mm)
% 2 = Peak acceleration (m/s^2)
% 3 = RMS acceleration (m/s^2)
% 4 = Suspension travel (mm)
% 5 = Tyre deflection (mm)

metrics = zeros(numCases,5);

for k = 1:numCases

    r = results(k);

    % Evaluate response from bump initiation onwards

    idx = t >= p.bumpStart;

    bodyDisp = max(abs(r.xs(idx)));

    peakAcc = max(abs(r.accs(idx)));

    rmsAcc = sqrt(mean(r.accs(idx).^2));

    travel = max(abs(r.xs(idx)-r.xu(idx)));

    tyre = max(abs(r.xu(idx)-r.road(idx)));

    maxForce = max(abs(r.force(idx)));

    metrics(k,:) = [ ...
        bodyDisp*1000, ...
        peakAcc, ...
        rmsAcc, ...
        travel*1000, ...
        tyre*1000];

    results(k).maxTyre = tyre*1000;

    results(k).maxForce = maxForce;

    fprintf('%-12s %12.3f %12.3f %12.3f %12.3f\n', ...
        caseNames{k}, ...
        bodyDisp*1000, ...
        peakAcc, ...
        rmsAcc, ...
        travel*1000);

end

%% 9. CONTROLLER CURRENT STATISTICS

fprintf('\n============================================\n');
fprintf('CONTROLLER CURRENT STATISTICS\n');
fprintf('============================================\n');

for k = 7:8

    r = results(k);

    idx = t >= p.bumpStart;

    fprintf('\n%s\n',caseNames{k});

    fprintf('Average current (entire simulation): %.4f A\n', ...
        mean(r.current));

    fprintf('Average current (after bump): %.4f A\n', ...
        mean(r.current(idx)));

    fprintf('Maximum actual current: %.4f A\n', ...
        max(r.current));

    fprintf('Maximum commanded current: %.4f A\n', ...
        max(r.command));

    fprintf('Maximum damper force: %.2f N\n', ...
        r.maxForce);

end

%% 10. SEMI-ACTIVE DISSIPATION CHECK

fprintf('\n============================================\n');
fprintf('SEMI-ACTIVE DISSIPATION CHECK\n');
fprintf('============================================\n');

for k = 1:numCases

    r = results(k);

    % Positive power indicates mechanical energy dissipation

    power = r.force .* r.vrel;

    fprintf('%-12s Minimum power = %.6f W\n', ...
        caseNames{k},min(power));

end

%% 11. CONTROLLER PERFORMANCE IMPROVEMENT

passiveRMS = metrics(1,3);

skyRMS = metrics(7,3);

pidRMS = metrics(8,3);

fprintf('\n============================================\n');
fprintf('RMS ACCELERATION IMPROVEMENT\n');
fprintf('============================================\n');

fprintf('Skyhook improvement: %.2f %%\n', ...
    (passiveRMS-skyRMS)/passiveRMS*100);

fprintf('PID improvement: %.2f %%\n', ...
    (passiveRMS-pidRMS)/passiveRMS*100);

%% 12. GRAPH 1: BODY ACCELERATION

figure('Name','Body Acceleration','Color','w');

hold on;

for k = 1:numCases

    plot(t,results(k).accs,'LineWidth',1.3);

end

title('Body Acceleration: Passive vs Controlled');

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

legend(caseNames,'Location','best');

grid on;
xlim([0 T]);

%% 13. GRAPH 2: BODY DISPLACEMENT

figure('Name','Body Displacement','Color','w');

hold on;

for k = 1:numCases

    plot(t,results(k).xs*1000,'LineWidth',1.3);

end

title('Body Displacement: Passive vs Controlled');

xlabel('Time (s)');
ylabel('Body Displacement (mm)');

legend(caseNames,'Location','best');

grid on;
xlim([0 T]);

%% 14. GRAPH 3: DAMPER FORCE
% Includes Passive, MR 2 A, Skyhook and PID

figure('Name','Damper Force','Color','w');

hold on;

for k = [1 6 7 8]

    plot(t,results(k).force,'LineWidth',1.5);

end

title('Damper Force Under Different Control Strategies');

xlabel('Time (s)');
ylabel('Damper Force (N)');

legend('Passive','MR 2 A','Skyhook','PID', ...
       'Location','best');

grid on;
xlim([0 T]);

%% 15. GRAPH 4: DYNAMIC MR CURRENT

figure('Name','Dynamic MR Current','Color','w');

hold on;

plot(t,results(7).current,'LineWidth',1.5);

plot(t,results(8).current,'LineWidth',1.5);

title('Dynamic MR Damper Current');

xlabel('Time (s)');
ylabel('Current (A)');

legend('Skyhook','PID','Location','best');

ylim([0 2.1]);

grid on;
xlim([0 T]);

%% 16. GRAPH 5: SUSPENSION TRAVEL

figure('Name','Suspension Travel','Color','w');

hold on;

for k = 1:numCases

    plot(t,(results(k).xs-results(k).xu)*1000, ...
        'LineWidth',1.3);

end

title('Suspension Travel Comparison');

xlabel('Time (s)');
ylabel('Suspension Travel (mm)');

legend(caseNames,'Location','best');

grid on;
xlim([0 T]);

%% 17. GRAPH 6: TYRE DEFLECTION

figure('Name','Tyre Deflection','Color','w');

hold on;

for k = 1:numCases

    plot(t,(results(k).xu-results(k).road)*1000, ...
        'LineWidth',1.3);

end

title('Tyre Deflection Comparison');

xlabel('Time (s)');
ylabel('Tyre Deflection (mm)');

legend(caseNames,'Location','best');

grid on;
xlim([0 T]);

%% 18. GRAPH 7: MR 2 A RELATIVE VELOCITY
% Diagnostic graph to investigate force oscillations

figure('Name','MR 2A Relative Velocity','Color','w');

plot(t,results(6).vrel,'LineWidth',1.3);

title('Relative Damper Velocity at 2 A');

xlabel('Time (s)');
ylabel('Relative Velocity (m/s)');

grid on;
xlim([0 T]);

%% 19. FINAL SIMULATION MESSAGE

fprintf('\n============================================\n');
fprintf('STAGE 4 SIMULATION COMPLETED SUCCESSFULLY\n');
fprintf('============================================\n');

%% ============================================================
% LOCAL FUNCTION 1: SUSPENSION SIMULATION
%% ============================================================

function r = simulateSuspension(t,dt,p,mode,fixedI)

N = length(t);

% State vector:
% x(1) = Sprung mass displacement
% x(2) = Sprung mass velocity
% x(3) = Unsprung mass displacement
% x(4) = Unsprung mass velocity
% x(5) = Actual MR damper current

x = zeros(5,1);

% Output storage

xs = zeros(1,N);
vs = zeros(1,N);

xu = zeros(1,N);
vu = zeros(1,N);

accs = zeros(1,N);

force = zeros(1,N);

current = zeros(1,N);
command = zeros(1,N);

road = zeros(1,N);

vrelArray = zeros(1,N);

% Controller internal states

integralError = 0;

skyMode = false;

previousCommand = 0;

%% TIME INTEGRATION LOOP

for j = 1:N

    now = t(j);

    % Road input

    road(j) = roadInput(now,p);

    % Relative damper velocity

    vrel = x(2)-x(4);

    %% CONTROLLER SELECTION

    if strcmp(mode,'passive')

        Icmd = 0;

    elseif strcmp(mode,'fixed')

        Icmd = fixedI;

    elseif strcmp(mode,'skyhook')

        % Skyhook switching signal

        switchingSignal = x(2)*vrel;

        % Hysteresis switching

        if switchingSignal > p.skyHyst

            skyMode = true;

        elseif switchingSignal < -p.skyHyst

            skyMode = false;

        end

        % Deadband

        if abs(vrel) < p.vDeadband

            Icmd = 0;

        elseif skyMode

            % Desired force on sprung mass

            Fdesired = -p.Csky*x(2);

            Icmd = mapForce(Fdesired,vrel,p);

        else

            Icmd = 0;

        end

    elseif strcmp(mode,'pid')

        % Desired body displacement = 0

        error = -x(1);

        % Desired force on sprung mass

        Fdesired = p.Kp*error ...
                 - p.Kd*x(2) ...
                 + p.Ki*integralError;

        % Semi-active feasibility

        feasible = Fdesired*vrel <= 0;

        if feasible

            % Candidate integral update

            integralCandidate = integralError + error*dt;

            integralCandidate = min(max(integralCandidate, ...
                -p.ItermLimit),p.ItermLimit);

            % Candidate desired force

            Fcandidate = p.Kp*error ...
                       - p.Kd*x(2) ...
                       + p.Ki*integralCandidate;

            Icandidate = mapForce(Fcandidate,vrel,p);

            % Anti-windup at current limits

            atUpperLimit = Icandidate >= p.Imax-1e-6;

            atLowerLimit = Icandidate <= p.Imin+1e-6;

            drivesFurtherIntoLimit = ...
                (atUpperLimit && error*Fcandidate > 0) || ...
                (atLowerLimit && error*Fcandidate < 0);

            if ~drivesFurtherIntoLimit

                integralError = integralCandidate;

            end

        end

        % Recalculate desired force

        Fdesired = p.Kp*error ...
                 - p.Kd*x(2) ...
                 + p.Ki*integralError;

        Icmd = mapForce(Fdesired,vrel,p);

    else

        error('Unknown controller mode');

    end

    %% CURRENT LIMITS AND SLEW RATE

    Icmd = min(max(Icmd,p.Imin),p.Imax);

    maxChange = p.maxSlew*dt;

    Icmd = min(max(Icmd, ...
        previousCommand-maxChange), ...
        previousCommand+maxChange);

    previousCommand = Icmd;

    command(j) = Icmd;

    %% STORE CURRENT STATES

    xs(j) = x(1);
    vs(j) = x(2);

    xu(j) = x(3);
    vu(j) = x(4);

    current(j) = x(5);

    vrelArray(j) = vs(j)-vu(j);

    %% CALCULATE DAMPER FORCE

    if strcmp(mode,'passive')

        % Independent passive damper model

        force(j) = p.cPassive*vrelArray(j);

    else

        % MR damper force using actual current

        force(j) = mrForce(vrelArray(j),current(j),p);

    end

    %% BODY ACCELERATION

    accs(j) = (-p.ks*(xs(j)-xu(j))-force(j))/p.ms;

    %% RK4 INTEGRATION

    if j < N

        f = @(time,state) systemODE( ...
            time,state,Icmd,p,mode);

        k1 = f(now,x);

        k2 = f(now+dt/2,x+dt*k1/2);

        k3 = f(now+dt/2,x+dt*k2/2);

        k4 = f(now+dt,x+dt*k3);

        x = x + dt*(k1+2*k2+2*k3+k4)/6;

    end

end

%% RETURN RESULTS

r.xs = xs;
r.vs = vs;

r.xu = xu;
r.vu = vu;

r.accs = accs;

r.force = force;

r.current = current;
r.command = command;

r.road = road;

r.vrel = vrelArray;

end

%% ============================================================
% LOCAL FUNCTION 2: QUARTER-CAR EQUATIONS OF MOTION
%% ============================================================

function dx = systemODE(t,x,Icmd,p,mode)

xs = x(1);
vs = x(2);

xu = x(3);
vu = x(4);

I = x(5);

road = roadInput(t,p);

vrel = vs-vu;

% Select damper model

if strcmp(mode,'passive')

    Fd = p.cPassive*vrel;

else

    Fd = mrForce(vrel,I,p);

end

% Sprung mass acceleration

as = (-p.ks*(xs-xu)-Fd)/p.ms;

% Unsprung mass acceleration

au = (p.ks*(xs-xu)+Fd-p.kt*(xu-road))/p.mu;

% First-order MR damper current response

dI = (Icmd-I)/p.tauI;

% State derivatives

dx = [vs; as; vu; au; dI];

end

%% ============================================================
% LOCAL FUNCTION 3: BINGHAM MR DAMPER MODEL
%% ============================================================

function F = mrForce(v,I,p)

% Current-dependent damping coefficient

c = p.c0+p.cSlope*I;

% Current-dependent yield force

Fy = p.FySlope*I;

% Smooth Bingham model

F = c*v+Fy*tanh(v/p.vEps);

end

%% ============================================================
% LOCAL FUNCTION 4: SEMI-ACTIVE FORCE-TO-CURRENT MAPPING
%% ============================================================

function Ibest = mapForce(Fdesired,vrel,p)

% Damper force acts opposite to the force
% applied to the sprung mass.

% Semi-active feasibility condition

if abs(vrel) < p.vDeadband || Fdesired*vrel > 0

    Ibest = p.Imin;

    return;

end

% Available current values

currents = p.Imin:p.Istep:p.Imax;

% Calculate achievable forces

Fd = zeros(size(currents));

for n = 1:length(currents)

    Fd(n) = mrForce(vrel,currents(n),p);

end

% Force acting on sprung mass

achievableForce = -Fd;

% Find current producing closest desired force

[~,idx] = min(abs(achievableForce-Fdesired));

Ibest = currents(idx);

end

%% ============================================================
% LOCAL FUNCTION 5: ROAD PROFILE
%% ============================================================

function z = roadInput(t,p)

elapsed = t-p.bumpStart;

if elapsed >= 0 && elapsed <= p.bumpDuration

    % Half-sine road bump

    z = p.bumpHeight*sin(pi*elapsed/p.bumpDuration);

else

    z = 0;

end

end
