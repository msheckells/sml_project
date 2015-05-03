function f = quad_rl()
%QUAD_BS Demonstration of nonlinear trajectory tracking control of a quadrotor system using backstepping controller
% systems such as a quadrotor. 
%
% See paper M. Kobilarov, "Trajectory tracking of a class of underactuated
% systems with external disturbances", 2013
%
% Author: Marin Kobilarov, marin(at)jhu.edu, 2013
%
%  Modified : Gowtham G


clear 

m = 1.316;
J=[0.0128;0.0128;0.0128];

% system parameters
S.m = m;
S.ag = [0;0;-9.81];
S.J = J;
S.e = [0;0;1];

% initial state
s.p = [0;0;0.182466];
rpy = [0;0;0];%Initial roll pitch yaw
initialquat = rpy2quat(rpy);
s.R = quat2mat(initialquat);
s.w = zeros(3,1);
s.v = zeros(3,1);
s.u = -S.ag'*S.e*S.m;







% timing
h = .01;
tf = 1000*h;

pds = [];
ps = [];
Rs = [];
vds = [];
vs = [];
ws = [];
us = [];
uns = [];
LFs = [];
Ts = [];
ts = [];




for t=h:h:tf,
    
  % computing tracking controls
  [s, u, T] = track(s, S);

  % update current state based on given controls
  s = update(s, h, u, T);
  % record states, etc...
  if (t > h)
    Ts = [Ts, T];
    us = [us, s.u];
    
    ws = [ws, s.w];

    vs = [vs, s.v];
    vds = [vds, sd.dp];

    pds = [pds, sd.p];
    ps = [ps, s.p];

    ts = [ts, t];
    LFs = [LFs, LF];    
  end
  
end


lw = 3;
fs = 15;

plot3(ps(1,:), ps(2,:), ps(3,:), 'b-','LineWidth',3)
hold on
plot3(pds(1,1:5:end), pds(2,1:5:end), pds(3,1:5:end), 'r--','LineWidth',3)
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
plot(ts, vds(1,:), 'r--','LineWidth',lw);
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
plot(ts, vs(2,:), 'b','LineWidth',lw)
hold on
plot(ts, vds(2,:), 'r--','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('$\dot y$','$\dot y_d$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,3)
plot(ts, vs(3,:), 'b','LineWidth',lw)
hold on
plot(ts, vds(3,:), 'r--','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('m/s')
h = legend('$\dot z$','$\dot z_d$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,4)
plot(ts, ws(1,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('rad/s')
h = legend('${\omega}_x$')
set(h,'Interpreter','latex')
set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,5)
plot(ts, ws(2,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('rad/s')
h = legend('${\omega}_y$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,6)
plot(ts, ws(3,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('rad/s')
h = legend('${\omega}_z$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,7)
plot(ts, Ts(1,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('Nm')
h = legend('${\mathbf\tau}_x$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,8)
plot(ts, Ts(2,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('${\mathbf\tau}_y$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,9)
plot(ts, Ts(3,:), 'b-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
h = legend('${\mathbf\tau}_z$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);


subplot(5,3,10)
plot(ts, us, '-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
ylabel('N')
h = legend('$u_{commanded}$')
set(h,'Interpreter','latex')

set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

subplot(5,3,11)
if ~isempty(uns)
  plot(ts, uns, '-','LineWidth',lw);
  set(gca, 'FontSize',fs)
  xlabel('sec.')
  ylabel('N')
  h = legend('$u_{actual}$')
  set(h,'Interpreter','latex')
  
  set(gca, 'Position', get(gca, 'OuterPosition') - ...
           get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
end

subplot(5,3,12)
plot(ts, LFs, '-','LineWidth',lw);
set(gca, 'FontSize',fs)
xlabel('sec.')
%ylabel('N')
h = legend('$V$')
set(h,'Interpreter','latex')



set(gca, 'Position', get(gca, 'OuterPosition') - ...
    get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);

function r = reward(s1, s2, u, S)

s1_all = [R2rpy(s.R); s1.p; s1.w; s1.v];
s2_all = [R2rpy(s.R); s2.p; s2.w; s2.v];
ds = s1_all - s2_all;

r_raw = -ds'*S.W*ds - u'*S.R*u;
r = (1-S.gamma)*tanh(r_raw);



function s = update(s, h, u, T)
% update s.u using s.du using an Euler-step

dw = (cross(S.J.*s.w, s.w) + T)./S.J;
dv = (S.ag + s.R*S.e*u)./S.m;

s.w = s.w + h*dw;
s.v = s.v + h*dv;

s.R = s.R*so3_exp(h*s.w);
s.p = s.p + h*s.v;

function rpy = R2rpy(R)
q = qGetQ(R);
rpy = quat2rpy(q)

function R = rpy2R(rpy)
q = rpy2quat(rpy);
R = qGetR(q);

function f = so3_exp(w, varargin)

theta = sqrt(w'*w);

if ~isnumeric(w(1)) && theta == sym('0')
  f = sym(eye(3));
  return
end

if (isnumeric(w(1)) && theta < eps)
  f = eye(3);
  return
end


%if (theta < eps)
%  f = eye(3);
%  return
%end



w_ = w/theta;
wh = so3_ad(w_);

f = eye(3) + wh*sin(theta) + wh*wh*(1-cos(theta));

if (~isnumeric(w(1)))
  for i=1:length(w),
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
    f = subs(f, {['(' char(w(i)) '^2)^(1/2)']}, {char(w(i))});
  end
end


function f = so3_ad(w)
f=[0, -w(3), w(2);
   w(3), 0, -w(1);
   -w(2), w(1), 0];

function out = unit(in)
if norm(in) > 0.01
    out = in / norm(in);
else
    out = in;
end
