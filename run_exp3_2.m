clc; clear; close all;
%% input
% 航路
formation = [0,0,5,20,2];
% 圆形编队 = [圆心x，圆心y，高度z，半径R，速度标量v] (5*1维)

%% Object properties
% Init position
y1_init = 0; % leader
y2_init = 5; % follower1
y3_init = -5; % follower2

% Body
g = 9.8;
m = 0.5; % 【飞机质量】
L = 0.2; % m 【机臂长度】
Ixx = 0.114; %【转动惯量】
Iyy = 0.114; %【转动惯量】
Izz = 0.158; %【转动惯量】

% Motor
k_F = 6.11*10^(-8)*3600/(4*pi^2); %N/(rad/s)^2 【电机转动力系数】
k_M = 1.5*10^(-9)*3600/(4*pi^2); %N*m/(rad/s)^2 【电机转动力矩系数】
k_m = 20; % 【电机响应延时（0.05s）】

%% formation tracker
threshold = 2; % 航点到达判定阈值

%% Path tracker
kct_p = 1;
kct_i = 0.05;
kct_d = 1;
kat_p = 1;
kat_i = 0.05;

%% Position Controller
kp_x = 1; ki_x = 0.01; kd_x = 1.0;
kp_y = 1; ki_y = 0.01; kd_y = 1.0;
kp_z = 1.2; ki_z = 1e-6; kd_z = 2.0;

%% Attitude controller
kp_phi = 2000; kd_phi = 4000;
kp_theta = 2000; kd_theta = 4000;
kp_psi = 800; kd_psi = 4000;

%% Run simulator
simtime = 50;
h = sim('exp3_2_formation_circle',simtime);
t = h.tout;

quad1.position = h.quad1(:,1:3);
quad1.velocity = h.quad1(:,4:6);
quad1.v_scalar = sqrt(sum(quad1.velocity.^2,2));
quad1.angle = h.quad1(:,7:9);

quad2.position = h.quad2(:,1:3);
quad2.velocity = h.quad2(:,4:6);
quad2.v_scalar = sqrt(sum(quad2.velocity.^2,2));
quad2.angle = h.quad2(:,7:9);

quad3.position = h.quad3(:,1:3);
quad3.velocity = h.quad3(:,4:6);
quad3.v_scalar = sqrt(sum(quad3.velocity.^2,2));
quad3.angle = h.quad3(:,7:9);

%% Dynamic plot
figure;
x = formation(1);
y = formation(2);
R = formation(4);
rectangle('Position',[x-R,y-R,2*R,2*R],'Curvature',[1,1],'linewidth',1,'linestyle','--'); hold on;

rectangle('Position',[x-1.5*R,y-1.5*R,3*R,3*R],'Curvature',[1,1],'edgecolor','white'); hold on;

for i = 1:5:size(t,1)
    p1(i) = scatter(quad1.position(i,1),quad1.position(i,2),'r'); hold on;
    p2(i) = scatter(quad2.position(i,1),quad2.position(i,2),'g'); hold on;
    p3(i) = scatter(quad3.position(i,1),quad3.position(i,2),'b'); hold on;
    xlabel('X/m'); ylabel('Y/m');
    axis equal;
    pause(0.001);
    if i > 30
        delete(p1(i-30)); delete(p2(i-30)); delete(p3(i-30));
    end
end

%% Plot
figure;
plot(quad1.position(:,1),quad1.position(:,2),'r',...
    quad2.position(:,1),quad2.position(:,2),'g',...
    quad3.position(:,1),quad3.position(:,2),'b'); hold on;

xlabel('X/m'); ylabel('Y/m');
legend('quad1','quad2','quad3');
axis equal;
grid on;
set(gca,'LooseInset',get(gca,'TightInset'));