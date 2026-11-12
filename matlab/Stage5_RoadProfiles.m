
%% ============================================================
% STAGE 5: ROAD PROFILE SIMULATIONS
% MR DAMPER QUARTER-CAR SUSPENSION MODEL
%
% Objectives:
% 1. Study different road bump heights.
% 2. Study different vehicle speeds.
% 3. Study random road excitation.
% 4. Compare Passive, Fixed MR, Skyhook and PID.
%
% Numerical method: RK4
% MATLAB R2016b or newer
%% ============================================================

clear;
clc;
close all;

rng(1);  % Reproducible random road

%% 1. QUARTER-CAR PARAMETERS

p.ms = 300;
p.mu = 40;

p.ks = 18000;
p.kt = 180000;

%% 2. MR DAMPER PARAMETERS

p.c0 = 800;
p.cSlope = 2100;
p.FySlope = 500;

p.Imin = 0;
p.Imax = 2;

p.Istep = 0.02;

p.vEps = 0.005;

p.tauI = 0.03;
p.maxSlew = 100;

%% 3. PASSIVE DAMPER

p.cPassive = 1000;

%% 4. CONTROLLER PARAMETERS

% Skyhook

p.Csky = 2500;
p.skyHyst = 0.002;
p.vDeadband = 0.002;

% PID

p.Kp = 18000;
p.Ki = 1000;
p.Kd = 3500;

p.ItermLimit = 0.15;

%% 5. SIMULATION PARAMETERS

dt = 0.001;

fixedCurrent = 0.5;

% Road bump dimensions

bumpLength = 2.0;       % Bump length in metres
bumpStart = 1.0;        % Bump begins at 1 second

%% 6. DEFINE SUSPENSION CONFIGURATIONS

modeNames = { ...
    'Passive', ...
    'MR 0.5 A', ...
    'Skyhook', ...
    'PID'};

numModes = length(modeNames);

%% ============================================================
% 7. DEFINE ROAD TEST CASES
%% ============================================================

cases = struct([]);

% CASE 1: BASELINE BUMP

cases(1).name = 'Baseline Bump';
cases(1).type = 'bump';
cases(1).height = 0.05;
cases(1).duration = 0.2;
cases(1).start = bumpStart;
cases(1).T = 5;

% CASE 2: BUMP HEIGHT = 30 MM

cases(2).name = '30 mm Bump';
cases(2).type = 'bump';
cases(2).height = 0.03;
cases(2).duration = bumpLength/(20/3.6);
cases(2).start = bumpStart;
cases(2).T = 5;

% CASE 3: BUMP HEIGHT = 50 MM

cases(3).name = '50 mm Bump';
cases(3).type = 'bump';
cases(3).height = 0.05;
cases(3).duration = bumpLength/(20/3.6);
cases(3).start = bumpStart;
cases(3).T = 5;

% CASE 4: BUMP HEIGHT = 80 MM

cases(4).name = '80 mm Bump';
cases(4).type = 'bump';
cases(4).height = 0.08;
cases(4).duration = bumpLength/(20/3.6);
cases(4).start = bumpStart;
cases(4).T = 5;

% CASE 5: VEHICLE SPEED = 10 KM/H

cases(5).name = 'Speed 10 km/h';
cases(5).type = 'bump';
cases(5).height = 0.05;
cases(5).duration = bumpLength/(10/3.6);
cases(5).start = bumpStart;
cases(5).T = 5;

% CASE 6: VEHICLE SPEED = 20 KM/H

cases(6).name = 'Speed 20 km/h';
cases(6).type = 'bump';
cases(6).height = 0.05;
cases(6).duration = bumpLength/(20/3.6);
cases(6).start = bumpStart;
cases(6).T = 5;

% CASE 7: VEHICLE SPEED = 30 KM/H

cases(7).name = 'Speed 30 km/h';
cases(7).type = 'bump';
cases(7).height = 0.05;
cases(7).duration = bumpLength/(30/3.6);
cases(7).start = bumpStart;
cases(7).T = 5;

% CASE 8: RANDOM ROAD - ISO 8608 INSPIRED CLASS C

cases(8).name = 'Random Road C';
cases(8).type = 'random';
cases(8).speed = 20/3.6;
cases(8).T = 10;

numCases = length(cases);

%% ============================================================
% 8. RUN ALL SIMULATIONS
%% ============================================================

results = cell(numCases,numModes);

metrics = zeros(numCases,numModes,5);

fprintf('\n============================================\n');
fprintf('STAGE 5: ROAD PROFILE SIMULATIONS\n');
fprintf('============================================\n');

for c = 1:numCases

    fprintf('\nRoad test: %s\n',cases(c).name);

    T = cases(c).T;

    t = 0:dt:T;

    % Prepare road profile

    if strcmp(cases(c).type,'random')

        road = makeRandomRoad(t,cases(c).speed);

    else

        road = cases(c);

    end

    for m = 1:numModes

        fprintf('Running %s...\n',modeNames{m});

        if m == 1

            mode = 'passive';
            current = 0;

        elseif m == 2

            mode = 'fixed';
            current = fixedCurrent;

        elseif m == 3

            mode = 'skyhook';
            current = 0;

        else

            mode = 'pid';
            current = 0;

        end

        r = simulateSuspension(t,dt,p,road,mode,current);

        results{c,m} = r;

        % Performance metrics

        if strcmp(road.type,'random')

            idx = true(size(t));

        else

            idx = t >= road.start;
        end

        bodyDisp = max(abs(r.xs(idx)));

        peakAcc = max(abs(r.accs(idx)));

        rmsAcc = sqrt(mean(r.accs(idx).^2));

        travel = max(abs(r.xs(idx)-r.xu(idx)));

        tyre = max(abs(r.xu(idx)-r.road(idx)));

        metrics(c,m,:) = [ ...
            bodyDisp*1000, ...
            peakAcc, ...
            rmsAcc, ...
            travel*1000, ...
            tyre*1000];

    end

end

%% ============================================================
% 9. PERFORMANCE COMPARISON TABLE
%% ============================================================

fprintf('\n============================================\n');
fprintf('STAGE 5 PERFORMANCE SUMMARY\n');
fprintf('============================================\n');

for c = 1:numCases

    fprintf('\nROAD TEST: %s\n',cases(c).name);

    fprintf('%-12s %9s %9s %9s %10s %10s\n', ...
        'System','Disp','PeakAcc','RMSAcc','Travel','Tyre');

    fprintf('%s\n',repmat('-',1,68));

    for m = 1:numModes

        v = squeeze(metrics(c,m,:));

        fprintf('%-12s %9.2f %9.3f %9.3f %10.2f %10.2f\n', ...
            modeNames{m},v(1),v(2),v(3),v(4),v(5));

    end

end

%% ============================================================
% 10. CONTROLLER CURRENT STATISTICS
%% ============================================================

fprintf('\n============================================\n');
fprintf('CONTROLLER CURRENT STATISTICS\n');
fprintf('============================================\n');

for c = 1:numCases

    fprintf('\n%s\n',cases(c).name);

    for m = 3:4

        r = results{c,m};

        fprintf('%s: Average = %.4f A, Maximum = %.4f A\n', ...
            modeNames{m},mean(r.current),max(r.current));

    end

end

%% ============================================================
% 11. GRAPH: BASELINE BODY ACCELERATION
%% ============================================================

figure('Name','Baseline Acceleration','Color','w');

hold on;

for m = 1:numModes

    r = results{1,m};

    plot(r.t,r.accs,'LineWidth',1.4);

end

title('Baseline Bump: Body Acceleration');

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

legend(modeNames,'Location','best');

grid on;

%% ============================================================
% 12. GRAPH: BASELINE BODY DISPLACEMENT
%% ============================================================

figure('Name','Baseline Displacement','Color','w');

hold on;

for m = 1:numModes

    r = results{1,m};

    plot(r.t,r.xs*1000,'LineWidth',1.4);

end

title('Baseline Bump: Body Displacement');

xlabel('Time (s)');
ylabel('Displacement (mm)');

legend(modeNames,'Location','best');

grid on;

%% ============================================================
% 13. GRAPH: BUMP HEIGHT COMPARISON
%% ============================================================

figure('Name','Bump Height Comparison','Color','w');

hold on;

for c = 2:4

    r = results{c,3};

    plot(r.t,r.xs*1000,'LineWidth',1.4);

end

title('Skyhook Response at Different Bump Heights');

xlabel('Time (s)');
ylabel('Body Displacement (mm)');

legend('30 mm','50 mm','80 mm','Location','best');

grid on;

%% ============================================================
% 14. GRAPH: VEHICLE SPEED COMPARISON
%% ============================================================

figure('Name','Vehicle Speed Comparison','Color','w');

hold on;

for c = 5:7

    r = results{c,3};

    plot(r.t,r.accs,'LineWidth',1.4);

end

title('Skyhook Acceleration at Different Speeds');

xlabel('Time (s)');
ylabel('Body Acceleration (m/s^2)');

legend('10 km/h','20 km/h','30 km/h', ...
    'Location','best');

grid on;

%% ============================================================
% 15. GRAPH: RANDOM ROAD PROFILE
%% ============================================================

figure('Name','Random Road Profile','Color','w');

plot(0:dt:cases(8).T, ...
    results{8,1}.road,'LineWidth',1.1);

title('Random Road Excitation - Class C');

xlabel('Time (s)');
ylabel('Road Displacement (m)');

grid on;

%% ============================================================
% 16. GRAPH: RANDOM ROAD BODY ACCELERATION
%% ============================================================

figure('Name','Random Road Acceleration','Color','w');

hold on;

for m = 1:numModes

    r = results{8,m};

    plot(r.t,r.accs,'LineWidth',1.1);

end

title('Random Road: Body Acceleration Comparison');

xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');

legend(modeNames,'Location','best');

grid on;

%% ============================================================
% 17. GRAPH: RMS ACCELERATION COMPARISON
%% ============================================================

figure('Name','RMS Acceleration Comparison','Color','w');

bar(metrics(:,:,3));

title('RMS Body Acceleration Across Road Conditions');

xlabel('Road Test');
ylabel('RMS Acceleration (m/s^2)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% ============================================================
% 18. GRAPH: SUSPENSION TRAVEL COMPARISON
%% ============================================================

figure('Name','Suspension Travel Comparison','Color','w');

bar(metrics(:,:,4));

title('Peak Suspension Travel Across Road Conditions');

xlabel('Road Test');
ylabel('Suspension Travel (mm)');

set(gca,'XTick',1:numCases);
set(gca,'XTickLabel',{cases.name});

xtickangle(25);

legend(modeNames,'Location','best');

grid on;

%% ============================================================
% 19. SAVE RESULTS
%% ============================================================

save('Stage5_Results.mat', ...
    'results','metrics','cases','modeNames','p','dt');

fprintf('\n============================================\n');
fprintf('STAGE 5 SIMULATIONS COMPLETED\n');
fprintf('Results saved to Stage5_Results.mat\n');
fprintf('============================================\n');

%% ============================================================
% LOCAL FUNCTION 1: QUARTER-CAR SIMULATION
%% ============================================================

function r = simulateSuspension(t,dt,p,road,mode,fixedI)

N = length(t);

% State vector:
% 1 = Sprung displacement
% 2 = Sprung velocity
% 3 = Unsprung displacement
% 4 = Unsprung velocity
% 5 = MR current

x = zeros(5,1);

xs = zeros(1,N);
vs = zeros(1,N);

xu = zeros(1,N);
vu = zeros(1,N);

accs = zeros(1,N);

force = zeros(1,N);

current = zeros(1,N);
command = zeros(1,N);

roadArray = zeros(1,N);
vrelArray = zeros(1,N);

integralError = 0;

skyMode = false;

previousCommand = 0;

for j = 1:N

    now = t(j);

    z = roadInput(now,road);

    vrel = x(2)-x(4);

    %% CONTROLLER

    if strcmp(mode,'passive')

        Icmd = 0;

    elseif strcmp(mode,'fixed')

        Icmd = fixedI;

    elseif strcmp(mode,'skyhook')

        switchingSignal = x(2)*vrel;

        if switchingSignal > p.skyHyst

            skyMode = true;

        elseif switchingSignal < -p.skyHyst

            skyMode = false;

        end

        if abs(vrel) < p.vDeadband

            Icmd = 0;

        elseif skyMode

            Fdesired = -p.Csky*x(2);

            Icmd = mapForce(Fdesired,vrel,p);

        else

            Icmd = 0;

        end

    elseif strcmp(mode,'pid')

        error = -x(1);

        Fdesired = p.Kp*error ...
                 - p.Kd*x(2) ...
                 + p.Ki*integralError;

        feasible = Fdesired*vrel <= 0;

        if feasible

            integralCandidate = integralError + error*dt;

            integralCandidate = min(max(integralCandidate, ...
                -p.ItermLimit),p.ItermLimit);

            Fcandidate = p.Kp*error ...
                       - p.Kd*x(2) ...
                       + p.Ki*integralCandidate;

            Icandidate = mapForce(Fcandidate,vrel,p);

            atUpper = Icandidate >= p.Imax-1e-6;
            atLower = Icandidate <= p.Imin+1e-6;

            drivesFurther = ...
                (atUpper && error*Fcandidate > 0) || ...
                (atLower && error*Fcandidate < 0);

            if ~drivesFurther

                integralError = integralCandidate;

            end

        end

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

    %% STORE STATES

    xs(j) = x(1);
    vs(j) = x(2);

    xu(j) = x(3);
    vu(j) = x(4);

    current(j) = x(5);

    command(j) = Icmd;

    roadArray(j) = z;

    vrelArray(j) = vrel;

    %% DAMPER FORCE

    if strcmp(mode,'passive')

        force(j) = p.cPassive*vrel;

    else

        force(j) = mrForce(vrel,x(5),p);

    end

    %% BODY ACCELERATION

    accs(j) = (-p.ks*(x(1)-x(3))-force(j))/p.ms;

    %% RK4 INTEGRATION

    if j < N

        f = @(time,state) systemODE( ...
            time,state,Icmd,p,road,mode);

        k1 = f(now,x);

        k2 = f(now+dt/2,x+dt*k1/2);

        k3 = f(now+dt/2,x+dt*k2/2);

        k4 = f(now+dt,x+dt*k3);

        x = x+dt*(k1+2*k2+2*k3+k4)/6;

    end

end

r.t = t;

r.xs = xs;
r.vs = vs;

r.xu = xu;
r.vu = vu;

r.accs = accs;

r.force = force;

r.current = current;
r.command = command;

r.road = roadArray;

r.vrel = vrelArray;

end

%% ============================================================
% LOCAL FUNCTION 2: EQUATIONS OF MOTION
%% ============================================================

function dx = systemODE(t,x,Icmd,p,road,mode)

xs = x(1);
vs = x(2);

xu = x(3);
vu = x(4);

I = x(5);

z = roadInput(t,road);

vrel = vs-vu;

if strcmp(mode,'passive')

    Fd = p.cPassive*vrel;

else

    Fd = mrForce(vrel,I,p);

end

as = (-p.ks*(xs-xu)-Fd)/p.ms;

au = (p.ks*(xs-xu)+Fd-p.kt*(xu-z))/p.mu;

dI = (Icmd-I)/p.tauI;

dx = [vs;as;vu;au;dI];

end

%% ============================================================
% LOCAL FUNCTION 3: MR DAMPER MODEL
%% ============================================================

function F = mrForce(v,I,p)

c = p.c0+p.cSlope*I;

Fy = p.FySlope*I;

F = c*v+Fy*tanh(v/p.vEps);

end

%% ============================================================
% LOCAL FUNCTION 4: FORCE TO CURRENT MAPPING
%% ============================================================

function Ibest = mapForce(Fdesired,vrel,p)

if abs(vrel) < p.vDeadband || Fdesired*vrel > 0

    Ibest = p.Imin;

    return;

end

currents = p.Imin:p.Istep:p.Imax;

Fd = zeros(size(currents));

for n = 1:length(currents)

    Fd(n) = mrForce(vrel,currents(n),p);

end

achievableForce = -Fd;

[~,idx] = min(abs(achievableForce-Fdesired));

Ibest = currents(idx);

end

%% ============================================================
% LOCAL FUNCTION 5: ROAD INPUT
%% ============================================================

function z = roadInput(t,road)

if strcmp(road.type,'bump')

    elapsed = t-road.start;

    if elapsed >= 0 && elapsed <= road.duration

        z = road.height*sin(pi*elapsed/road.duration);

    else

        z = 0;

    end

else

    z = interp1(road.t,road.z,t,'linear',0);

end

end

%% ============================================================
% LOCAL FUNCTION 6: RANDOM ROAD GENERATOR
%% ============================================================

function road = makeRandomRoad(t,speed)

% ISO 8608-inspired Class C road.
%
% Reference spatial frequency = 0.1 cycles/m
% Class C reference PSD = 256e-6 m^3
% PSD slope = -2
%
% Road profile is generated in the spatial domain
% and converted into time excitation using vehicle speed.

nMin = 0.05;
nMax = 10;

numFreq = 400;

freq = linspace(nMin,nMax,numFreq);

dn = freq(2)-freq(1);

n0 = 0.1;

G0 = 256e-6;

Gd = G0*(freq/n0).^(-2);

x = speed*t;

z = zeros(size(x));

phase = 2*pi*rand(1,numFreq);

for k = 1:numFreq

    amplitude = sqrt(2*Gd(k)*dn);

    z = z+amplitude*cos( ...
        2*pi*freq(k)*x+phase(k));

end

% Ensure zero initial road displacement

z = z-z(1);

road.type = 'random';

road.t = t;

road.z = z;

road.speed = speed;

end
