function f = car_rl()


clear 


% reward weights
s.W = diag([.1 .1 0 0]);
s.R = diag([.02 0.02]);

% initial state
s.x = [1;0; 0; 0];
s.u = [0.0; 0.0];
s.l = 1;
s.gamma = 0.8;
s.num_outputs = 9;

% WFNN params
s.eps = .001;
s.c = .01;

s.xd = [0; 0; 0; 0];

reward(s, s.xd, s.u)

% timing
s.h = .01;
tf = 10000*s.h;

% for plots
lw = 3;
fs = 15;

ps = [s.x(1:2)];
vs = [];
ws = [];
us = [];
ts = [];
explore_rate = 0.0;
nntrain = true;
nninit = false;

qnns = .1*rand(s.num_outputs,1) - 1; %-1*ones(s.num_outputs,1);
unns = .05*rand(length(s.u)*s.num_outputs,1) - .025;
qunns = [];
for i = 1:length(qnns)
    qunns = [qunns; unns(2*i-1:2*i); qnns(i)];
end

if nninit
    nn = feedforwardnet(12); 
    
    %hide window
    nn.trainParam.showWindow = false;
    
    nn.layers{2}.transferFcn='tansig';
    nn = configure(nn, s.x, qunns);
    %view(nn)
    save nn_particle.mat nn
else
    disp('Loading nn');
    load nn_particle.mat
end

figure, plot(ps(1,:), ps(2,:), 'b-','LineWidth',3)
hold on
%plot3(pds(1,1:5:end), pds(2,1:5:end), pds(3,1:5:end), 'r--','LineWidth',3)
set(gca, 'FontSize',fs)
xlabel('m')
ylabel('m')
zlabel('m')
axis equal

%%
for t=s.h:s.h:tf,
    [s.u, us, qs] = nncontrol(nn, s);
    if rand() < explore_rate
        s.u = 2.0*rand(2,1) - 1.0;
    end
    s.u

    xt = s.x;
    % update current state based on given controls
    s = update(s, s.h, s.u);
    
    if nntrain
        qv = Qval(s, nn);
        nnqv = nnQval(s.u, us, qs, s);
        dQ = qv - nnqv;
        dqs = getDqs(s.u, us, qs, s, dQ);
        dus = getDus(s.u, us, qs, s, dQ);
        newus = us + dus;
        newqs = qs' + dqs;
        
        uqs = [];
        
        for i=1:length(newqs)
            uqs = [uqs; newus(:,i); newqs(i)];
        end
        nn = train(nn, xt, uqs);
    end
    
    % record states, etc...
    if (t > s.h)
        ps = [ps, s.x(1:2)];
        us = [us, s.u];
        ws = [ws, s.x(3)];
        vs = [vs, s.x(4)];
        ts = [ts, t];
        
        plot(ps(1,:), ps(2,:), 'b-','LineWidth',3)
        if rem(t,.5) == 0
            drawnow
        end
        if rem(t,1.0) == 0
            save nn_particle.mat nn
        end
    end
    
end




plot(ps(1,:), ps(2,:), 'b-','LineWidth',3)
hold on
%plot3(pds(1,1:5:end), pds(2,1:5:end), pds(3,1:5:end), 'r--','LineWidth',3)
set(gca, 'FontSize',fs)
xlabel('m')
ylabel('m')
zlabel('m')
axis equal

h = legend('$\mathbf x$','$\mathbf x_d$')

set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


figure 

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,1)
plot(ts, vs(1,:), 'b','LineWidth',lw)
hold on
%plot(ts, vds(1,:), 'r--','LineWidth',lw);
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('m/s')
h = legend('$\dot x$','$\dot x_d$')
set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);



subplot(5,3,2)
plot(ts, ws(1,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('rad/s')
h = legend('${\omega}_x$')
set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);



subplot(5,3,3)
plot(ts, us(1,:), '-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('N')
h = legend('$u_{commanded}$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


function [u, us, qs] = nncontrol(nn, s)
urs = nn(s.x);
us = [];
qs = [];
best_r = urs(length(s.u)+1);
best_u = urs(1:length(s.u));
for i=length(s.u)+1:(1+length(s.u)):length(urs)
    qs = [qs, urs(i)];
    us = [us, urs(i-2:i-1)];
    if urs(i) > best_r
        best_r = urs(i);
        best_u = urs(i-2:i-1);
    end
end

u = best_u;


function dqs = getDqs(u, us, qs, s, dQ)
dqs = zeros(length(qs),1);
wsumv = wsum(u, us, qs, s);
unormv = unorm(u, us, qs, s);
for i=1:length(dqs)
    dqs(i) = dQ*unormv*(distance(u, us, qs, s, i) + qs(i)*s.c)-wsumv*s.c/(unormv*distance(u, us, qs, s, i))^2;
end

function dus = getDus(u, us, qs, s, dQ)
dus = zeros(size(us));
wsumv = wsum(u, us, qs, s);
unormv = unorm(u, us, qs, s);
for i=1:size(dus,1)
    for j=1:size(dus,2)
        dus(i,j) = dQ*(wsumv-unormv*qs(j))*2*(us(i,j)-u(i))/(unormv*distance(u, us, qs, s, j))^2;
    end
end

function Q = Qval(s, nn)
r = reward(s, s.xd, s.u)

% Get next Q
[nextu, us, qs] = nncontrol(nn, s);
snew = update(s, s.h, nextu);
Q = r + s.gamma*nnQval(nextu, us, qs, snew);

function Q = nnQval(u, us, qs, s)
Q = wsum(u, us, qs, s)/unorm(u, us, qs, s);

function sum = distance(u, us, qs, s, i)
qmax = max(qs);
sum = (norm(u - us(:,i))^2 + s.c*(qmax-qs(i)) + s.eps);

function sum = wsum(u, us, qs, s)
sum = 0;
qmax = max(qs);
for i=1:length(qs)
    sum = sum + qs(i)/(norm(u - us(:,i))^2 + s.c*(qmax-qs(i)) + s.eps);
end

function sum = unorm(u, us, qs, s)
sum = 0;
qmax = max(qs);
for i=1:length(qs)
    sum = sum + 1/(norm(u - us(:,i))^2 + s.c*(qmax-qs(i)) + s.eps);
end

function r = reward(s, xd, u)
dx = s.x - xd;

r_raw = -dx'*s.W*dx - u'*s.R*u;
r = -(1-s.gamma)*tanh(r_raw);

function s = update(s, h, u)
s.x(1:2) = s.x(1:2) + h*u;
 

