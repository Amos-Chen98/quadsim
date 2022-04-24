clc; clear; close all;
%% input
p_des = [10,10,-10];
state_track = [0,0,0,0;
                10,0,-10,1;
               10,10,-10,1;
               0,10,-10,1;
              0,0,0,1];
%% Object properties 
% Body 
g = 9.8;
m = 0.5; % 【飞机质量】
L = 0.2; % m 【机臂长度】
Ixx = 0.114; %【转动惯量】
Iyy = 0.114; %【转动惯量】
Izz = 0.158; %【转动惯量】
% I=[Ixx 0 0;0 Iyy 0;0 0 Izz];

% Motor
k_F = 6.11*10^(-8)*3600/(4*pi^2); %N/(rad/s)^2 【电机转动力系数】
k_M = 1.5*10^(-9)*3600/(4*pi^2); %N*m/(rad/s)^2 【电机转动力矩系数】
k_m = 20; % 【电机响应延时（0.05s）】

%% Path tracker
kct_p = 1;
kct_i = 0.05;
kct_d = 1;
kat_p = 1;
kat_i = 0.05;

%% Position Controller
kp_x = 0.5; ki_x = 3e-4; kd_x = 1.0;
kp_y = 0.5; ki_y = 3e-4; kd_y = 1.0;
kp_z = 1.2; ki_z = 1e-6; kd_z = 2.0;

%% Attitude controller
kp_phi = 2000; kd_phi = 4000;
kp_theta = 2000; kd_theta = 4000;
kp_psi = 800; kd_psi = 4000;

%% Run simulator
h = sim('exp2_tracking');
t = h.tout;
v = sqrt(sum(h.velocity.^2,2));

%% Plot
figure;
plot(h.position(:,1),h.position(:,2));
xlabel('X/m'); ylabel('Y/m');
axis equal;
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));


% figure;
% plot(t,h.position(:,1),t,h.position(:,2),t,h.position(:,3)); hold on;
% plot(t,v,'--');
% xlabel('t/s'); ylabel('Position/m');
% legend('X','Y','Z','v');
% grid on;
% set(gca,'LooseInset',get(gca,'TightInset'));
% 
% figure;
% plot(t,h.velocity(:,1),t,h.velocity(:,2),t,h.velocity(:,3));
% xlabel('t/s'); ylabel('V/(m/s)');
% legend('V_x','V_y','V_z');
% grid on;
% set(gca,'LooseInset',get(gca,'TightInset'));
% 
% figure;
% plot(t,h.angle(:,1),t,h.angle(:,2),t,h.angle(:,3));
% xlabel('t/s'); ylabel('Angle/rad');
% legend('\phi','\theta','\psi');
% grid on;
% set(gca,'LooseInset',get(gca,'TightInset'));