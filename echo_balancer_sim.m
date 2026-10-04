function echo_balancer_sim()
% ECHO_BALANCER_SIM  Self-balancing robot with a Kalman-NIS safety gate.
%
%   Two-wheeled inverted pendulum (NXTway-GS, Yamamoto 2008) balanced by an
%   LQR. A Kalman filter on the NOMINAL model produces the normalized
%   innovation squared NIS_k = nu' S^-1 nu ~ chi2(3). The windowed sum
%   eps_k (last N samples) drives a Normal -> Cautious -> Halt state machine
%   (hysteresis + minimum dwell). Three runs are shown:
%       1) nominal (gate must stay NORMAL)
%       2) gyro-bias sensor fault   (estimator/sensor mismatch)
%       3) surface change (wheel-floor friction step, plant mismatch)
%   Runs in MATLAB (no toolboxes) and GNU Octave.  Usage:  echo_balancer_sim

close all; clc;
P = plant_params();
dtc = 0.005;                 % control/estimation period [s] (200 Hz)
sub = 5;                     % RK4 sub-steps per control period (dt = 1 ms)
Tend = 30;                   % run length [s]

%% ---- 1. Linearization and LQR (state [theta psi thetadot psidot]) ------
[A, B] = linearize(P);
Q = diag([1 1e3 1 1]);  Rw = 1e2;
K = lqr_care(A, B, Q, Rw);
fprintf('Open-loop eig(A)  : %s\n', mat2str(sort(real(eig(A)))', 4));
fprintf('LQR gain K        : %s   (spec: -0.10 -49.3 -2.09 -4.55)\n', mat2str(K, 3));
fprintf('Closed-loop poles : %s\n\n', mat2str(sort(real(eig(A - B*K)))', 4));

%% ---- 2. Discrete Kalman filter model (ZOH, states + gyro bias) ---------
C = [1 -1 0 0; 0 0 0 1; 0 1 0 0];                 % y = [theta_enc; gyro; acc]
Ae = [A zeros(4,1); zeros(1,5)];  Be = [B; 0];
Md = expm([Ae Be; zeros(1,6)] * dtc);
Fd = Md(1:5,1:5);  Gd = Md(1:5,6);
Ce = [C(1,:) 0; C(2,:) 1; C(3,:) 0];              % gyro measures psidot + bias
S = struct('sg',0.005,'bw',1e-4,'sa',0.02,'cpr',360);  % sensor params (CLAUDE.md s7)
Qk = diag([1e-8 1e-8 1e-6 1e-6 (S.bw^2)*dtc]);
Rk = diag([(2*pi/S.cpr)^2/12, S.sg^2, S.sa^2]);

%% ---- 3. Gate calibration on nominal seeds (thresholds from data) -------
N = 200;                                          % window = 1 s
calMax = zeros(1,5);
for s = 1:5
    r = simulate(P,K,Fd,Gd,Ce,Qk,Rk,S,dtc,sub,Tend,N,[],[],s);
    calMax(s) = max(r.eps(N+1:end));
end
gate = struct('N',N,'tau1',1.25*max(calMax),'tau2',1.6*max(calMax),'Tdwell',0.5);
gate.tau1_exit = 0.8*gate.tau1;  gate.tau2_exit = 0.8*gate.tau2;
fprintf('Nominal max eps over 5 calibration seeds: %.0f -> tau1=%.0f, tau2=%.0f (N=%d)\n', ...
        max(calMax), gate.tau1, gate.tau2, N);
fprintf('(chi2(3N) 95%% quantile would be %.0f; NIS samples are correlated, so\n', ...
        3*N + 1.645*sqrt(2*3*N));
fprintf(' thresholds are calibrated empirically on nominal runs.)\n\n');

%% ---- 4. Scenarios ------------------------------------------------------
dist(1) = struct('name','Nominal (no disturbance)','t0',inf,'type','none','mag',0);
dist(2) = struct('name','Gyro bias fault (+0.15 rad/s @ 10 s)','t0',10,'type','gyro','mag',0.15);
dist(3) = struct('name','Surface change (f_w = 0.025 @ 10 s)','t0',10,'type','fw','mag',0.025);
res = cell(1,3);
for i = 1:3
    res{i} = simulate(P,K,Fd,Gd,Ce,Qk,Rk,S,dtc,sub,Tend,N,gate,dist(i),100+i);
    r = res{i};
    chg = find(diff(r.mode) ~= 0);
    fprintf('%-40s fell=%d  ', dist(i).name, r.fell);
    if isempty(chg), fprintf('no mode change\n');
    else
        fprintf('first non-NORMAL at t=%.2f s', r.t(find(r.mode>0,1)));
        if isfinite(dist(i).t0), fprintf(' (delay %.2f s)', r.t(find(r.mode>0,1))-dist(i).t0); end
        fprintf('\n');
    end
end

%% ---- 5. Plots ----------------------------------------------------------
mc = [0.8 1 0.8; 1 0.95 0.6; 1 0.7 0.7];          % NORMAL / CAUTIOUS / HALT shading
figure('Name','Echo Balancer','Position',[50 50 1100 800]);
for i = 1:3
    r = res{i};
    subplot(3,3,i);   shade(r.t, r.mode, mc, [-3 3]); hold on;
    plot(r.t, rad2deg(r.psi), 'k'); ylabel('\psi [deg]'); ylim([-3 3]);
    title(dist(i).name, 'FontSize', 8);
    subplot(3,3,3+i); shade(r.t, r.mode, mc, [0 max(2*gate.tau2, 1)]); hold on;
    plot(r.t, r.nis*N/3, 'Color', [.75 .75 .75]); % scaled for visibility only
    plot(r.t, r.eps, 'b'); line([0 Tend],[gate.tau1 gate.tau1],'Color',[.9 .6 0]);
    line([0 Tend],[gate.tau2 gate.tau2],'Color','r'); ylabel('\epsilon_k'); ylim([0 2*gate.tau2]);
    subplot(3,3,6+i); stairs(r.t, r.mode, 'k'); ylim([-.2 2.2]); yticks(0:2);
    yticklabels({'NORMAL','CAUTIOUS','HALT'}); xlabel('t [s]'); ylabel('mode');
end
drawnow;
end

%% =======================================================================
function r = simulate(P,K,Fd,Gd,Ce,Qk,Rk,S,dtc,sub,Tend,N,gate,dist,seed)
% One closed-loop episode. Returns time histories.
rng(seed);
nsteps = round(Tend/dtc);  h = dtc/sub;
x = [0; deg2rad(1); 0; 0];                        % small initial lean
xh = [0;0;0;0;0];  Pk = diag([1e-4 1e-4 1e-4 1e-4 1e-6]);
bg = 0;  mode = 0;  tmode = 0;  nisbuf = zeros(N,1);  nisN = 0;
vref = 0.3;  thref = 0;  u = 0;  fw = P.fw;  fell = false;
r.t = (0:nsteps-1)'*dtc; r.psi = zeros(nsteps,1); r.nis = r.psi; r.eps = r.psi;
r.mode = r.psi; r.theta = r.psi;
scale = [1 0.4 0];                                % speed multiplier per mode
for k = 1:nsteps
    t = (k-1)*dtc;
    % --- disturbance schedule
    gbias = 0;  P.fw = fw;
    if ~isempty(dist) && t >= dist.t0
        if strcmp(dist.type,'gyro'), gbias = dist.mag; end
        if strcmp(dist.type,'fw'),   P.fw  = dist.mag; end
    end
    % --- sensors (accelerometer uses true specific force, so it is
    %     corrupted by body acceleration during motion)
    [~, acc] = plant_f(x, u, P);
    ah = P.R*acc(1) + P.L*(acc(2)*cos(x(2)) - x(4)^2*sin(x(2)));
    av = P.L*(-acc(2)*sin(x(2)) - x(4)^2*cos(x(2)));
    ax = ah*cos(x(2)) - (av+P.g)*sin(x(2));
    az = ah*sin(x(2)) + (av+P.g)*cos(x(2));
    bg = bg + S.bw*sqrt(dtc)*randn;
    q = 2*pi/S.cpr;
    y = [round((x(1)-x(2))/q)*q; ...
         x(4) + bg + gbias + S.sg*randn; ...
         atan2(-ax, az) + S.sa*randn];
    % --- Kalman filter (nominal model) and NIS
    xp = Fd*xh + Gd*u;   Pp = Fd*Pk*Fd' + Qk;
    nu = y - Ce*xp;      Sk = Ce*Pp*Ce' + Rk;
    nis = nu'/Sk*nu;
    Kg = Pp*Ce'/Sk;      xh = xp + Kg*nu;   Pk = (eye(5)-Kg*Ce)*Pp;
    nisbuf = [nisbuf(2:end); nis];  nisN = min(nisN+1, N);
    ep = sum(nisbuf);
    % --- gate (Normal=0 / Cautious=1 / Halt=2)
    if ~isempty(gate) && nisN >= N
        tmode = tmode + dtc;
        if tmode >= gate.Tdwell
            new = mode;
            switch mode
                case 0, if ep > gate.tau2, new = 2; elseif ep > gate.tau1, new = 1; end
                case 1, if ep > gate.tau2, new = 2; elseif ep < gate.tau1_exit, new = 0; end
                case 2, if ep < gate.tau2_exit, new = 1; end
            end
            if new ~= mode, mode = new; tmode = 0; end
        end
    end
    % --- balance LQR with speed reference (HALT keeps balancing in place)
    vr = vref*scale(mode+1);  thref = thref + vr*dtc;
    u = -K*(xh(1:4) - [thref; 0; vr; 0]);
    u = max(min(u, 2*7.4), -2*7.4);               % v_l=v_r=u/2, |v| <= V_batt
    % --- plant: RK4 with zero-order-hold input
    for j = 1:sub
        k1 = plant_f(x,u,P); k2 = plant_f(x+h/2*k1,u,P);
        k3 = plant_f(x+h/2*k2,u,P); k4 = plant_f(x+h*k3,u,P);
        x = x + h/6*(k1+2*k2+2*k3+k4);
    end
    r.psi(k)=x(2); r.nis(k)=nis; r.eps(k)=ep; r.mode(k)=mode; r.theta(k)=x(1);
    if abs(x(2)) > deg2rad(45)                    % fall: motors off, episode ends
        fell = true;  r.t = r.t(1:k); r.psi = r.psi(1:k); r.nis = r.nis(1:k);
        r.eps = r.eps(1:k); r.mode = r.mode(1:k); r.theta = r.theta(1:k); break;
    end
end
r.fell = fell;
end

%% =======================================================================
function [dx, acc] = plant_f(x, u, P)
% Planar NXTway-GS dynamics (CLAUDE.md s5), x=[theta psi thetadot psidot],
% u = v_l + v_r. 2x2 coupled system solved with the backslash operator.
al = P.n*P.Kt/P.Rm;  be = P.n^2*P.Kt*P.Kb/P.Rm + P.fm;
psi = x(2); td = x(3); pd = x(4);
M2 = [(2*P.m+P.M)*P.R^2 + 2*P.Jw + 2*P.n^2*P.Jm,  P.M*P.L*P.R*cos(psi) - 2*P.n^2*P.Jm;
      P.M*P.L*P.R*cos(psi) - 2*P.n^2*P.Jm,         P.M*P.L^2 + P.Jpsi + 2*P.n^2*P.Jm];
Ft = al*u - 2*(be+P.fw)*td + 2*be*pd;
Fp = -al*u + 2*be*td - 2*be*pd;
rhs = [Ft + P.M*P.L*P.R*pd^2*sin(psi); Fp + P.M*P.g*P.L*sin(psi)];
acc = M2 \ rhs;
dx = [td; pd; acc];
end

function [A, B] = linearize(P)
% Central-difference Jacobian of plant_f at the upright equilibrium.
x0 = zeros(4,1); e = 1e-6; A = zeros(4);
for i = 1:4
    d = zeros(4,1); d(i) = e;
    A(:,i) = (plant_f(x0+d,0,P) - plant_f(x0-d,0,P))/(2*e);
end
B = (plant_f(x0,e,P) - plant_f(x0,-e,P))/(2*e);
end

function K = lqr_care(A, B, Q, R)
% LQR gain from the stable invariant subspace of the Hamiltonian matrix
% (toolbox-free replacement for lqr).
n = size(A,1);
H = [A, -B*(R\B'); -Q, -A'];
[V, D] = eig(H);
idx = real(diag(D)) < 0;
X1 = V(1:n, idx);  X2 = V(n+1:end, idx);
Pm = real(X2/X1);
K = R\(B'*Pm);
end

function P = plant_params()
P.g=9.81; P.m=0.03; P.R=0.04; P.Jw=P.m*P.R^2/2; P.M=0.6; P.W=0.14; P.D=0.04;
P.H=0.144; P.L=P.H/2; P.Jpsi=P.M*P.L^2/3; P.Jm=1e-5; P.Rm=6.69; P.Kb=0.468;
P.Kt=0.317; P.n=1; P.fm=0.0022; P.fw=0;
end

function shade(t, mode, mc, yl)
% Background colour = gate mode.
hold on;
for m = 0:2
    idx = (mode == m);
    if ~any(idx), continue; end
    d = diff([0; idx(:); 0]);  s = find(d==1);  e = find(d==-1)-1;
    for j = 1:numel(s)
        t1 = t(s(j));  t2 = t(min(e(j)+1, numel(t)));
        patch([t1 t2 t2 t1],[yl(1) yl(1) yl(2) yl(2)], mc(m+1,:), 'EdgeColor','none');
    end
end
xlim([t(1) t(end)]); ylim(yl);
end
