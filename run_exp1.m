clc; clear; close all;
%% input
p_des = [10,10,-10];
% x_des = 10;
% y_des = 10;
% z_des = -10;
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

%% Position Controller
kp_x = 0.5; ki_x = 3e-4; kd_x = 1.0;
kp_y = 0.5; ki_y = 3e-4; kd_y = 1.0;
kp_z = 1.2; ki_z = 1e-6; kd_z = 2.0;

%% Attitude controller
kp_phi = 2000; kd_phi = 4000;
kp_theta = 2000; kd_theta = 4000;
kp_psi = 800; kd_psi = 4000;

%% Run simulator
h = sim('exp1_hover');

%% Plot
t = h.tout;
figure;
plot(t,h.position(:,1),t,h.position(:,2),t,h.position(:,3));
legend('X','Y','Z');
xlabel('X/m'); ylabel('Y/m');
grid on;

figure;
plot(t,h.velocity(:,1),t,h.velocity(:,2),t,h.velocity(:,3));
grid on;
legend('V_x','V_y','V_z');

figure;
plot(t,h.angle(:,1),t,h.angle(:,2),t,h.angle(:,3));
grid on;
legend('\phi','\theta','\psi');