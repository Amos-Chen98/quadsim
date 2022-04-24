function draw_quad_traj_by_app( x, y, z, vx,vy,vz, roll ,pitch,yaw,isRecord,quadNum,app)
%x,y,z：位置，NED坐标系。如果是多架次飞行器的位置数据，则x(:,1)表示第1架飞行器的x位置，x(:,2)表示第2架飞行器的x位置...
%vx,vy,vz：速度，NED坐标系
%roll,pitch,yaw：姿态，NED坐标系
%isRecord:是否记录录像。=1记录，=0不记录
%quadNum：无人机的个数
%handles：GUI窗口句柄

%仿真坐标系为北-东-地，而绘制坐标系为北-西-天
local_yaw = pi/2-yaw;
local_pitch= -pitch;
local_roll = roll;
local_x = x;
local_y = y;
local_z = -z;  %原始坐标系为NED，绘图时Z轴向上
local_vx=vx;
local_vy=vy;
local_vz=-vz;
%
[max_x, index_max_x] = max(max(local_x));
[min_x, index_min_x] = min(min(local_x));
[max_y, index_max_y] = max(max(local_y));
[min_y, index_min_y] = min(min(local_y));
[max_z, index_max_z] = max(max(local_z));
[min_z, index_min_z] = min(min(local_z));

%
%清理3D视图内容
cla(app.fig_3d);
%将3D视图设置为当前待绘制的坐标轴
%axes(app.fig_3d);
% number of quadrotors
nquad = quadNum;

% max time
time_tol = size(local_x,1)*0.01;

%% **************************** FIGURES *****************************
fprintf('3D坐标初始化...\n')
%h_fig = app.fig_3d;
%h_3d = gca;
%允许在GUI界面上旋转3D界面
%rotate3d(app.fig_3d,"on");
axis(app.fig_3d,"equal");
grid(app.fig_3d, "on");
view(app.fig_3d,3);
xlabel(app.fig_3d,'x [m]'); 
ylabel(app.fig_3d,'y [m]'); 
zlabel(app.fig_3d,'z [m]')

%三维坐标区间
axis(app.fig_3d,[min_x*2-10,max_x*2+10,min_y*2-10,max_y*2+10,min_z,max_z*2+10])   
axis(app.fig_3d,"normal");
quadcolors = lines(nquad);

%set(gcf,'Renderer','OpenGL')
%% *********************** INITIAL CONDITIONS ***********************
fprintf('设置初始条件...\n')
max_iter  = size(local_x,1);      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
time      = starttime; % current time

%准备time=0时刻数据
for qn = 1:nquad
    % Get start and stop position
    desired_state.pos = [local_x(1,qn);local_y(1,qn);local_z(1,qn)];
    desired_state.vel = [local_vx(1,qn);local_vy(1,qn);local_vz(1,qn)];
    desired_state.acc = [0;0;0];
    desired_state.yaw = local_yaw(1,qn);
    desired_state.yawdot = 0;
    
    x0{qn} = init_state(desired_state.pos, local_roll(1,qn), local_pitch(1,qn), local_yaw(1,qn),...
        local_vx(1,qn), local_vy(1,qn), local_vz(1,qn));
end

x = x0;        % state

%% ************************* RUN SIMULATION *************************
%是否保存为录像
OUTPUT_TO_VIDEO = 0;
if isRecord == 1
    OUTPUT_TO_VIDEO = 1;
end
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('.\quad_traj.avi');
    open(v)
end

fprintf('运行仿真....')
% Main loop
for iter = 1:max_iter
    
    %time:当前时间
    timeint = time:tstep:time+cstep;

    %tic;
    
    %仪表只显示第1架四旋翼的信息
    app.speed_meter.Airspeed= sqrt(local_vx(iter,1)^2+local_vy(iter,1)^2+local_vz(iter,1)^2)*3.6;  %km/h
    app.horizon_meter.Pitch= rad2deg(local_pitch(iter,1));  %deg
    app.horizon_meter.Roll= rad2deg(local_roll(iter,1));    %deg
    app.heading_meter.Heading=rad2deg(local_yaw(iter,1));   %deg
    
    
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            %绘制第一次数据。QuadPlot是一个类
            QP{qn} = QuadPlot(qn, x0{qn}, 1, 0.1, 'r', max_iter, app.fig_3d);
            %准备第2帧数据
            desired_state.pos = [local_x(2,qn);local_y(2,qn);local_z(2,qn)];
            desired_state.vel = [local_vx(2,qn);local_vy(2,qn);local_vz(2,qn)];
            desired_state.acc = [0;0;0];
            desired_state.yaw = local_yaw(2,qn);
            desired_state.yawdot = 0;
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
           
        end
        %绘制第2次数据
        if iter == 2
            hold(QP{qn}.h_3d, 'on')
            plot3(QP{qn}.h_3d, QP{qn}.des_state(1), QP{qn}.des_state(2), QP{qn}.des_state(3), 'r.');
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[1 2]), ...
                QP{qn}.motor(2,[1 2]), ...
                QP{qn}.motor(3,[1 2]), ...
                'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 5);   
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[3 4]), ...
                QP{qn}.motor(2,[3 4]), ...
                QP{qn}.motor(3,[3 4]), ...
                'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[1 3]), ...
                QP{qn}.motor(2,[1 3]), ...
                QP{qn}.motor(3,[1 3]), ...
                '-', 'Color', 'k', 'LineWidth',1);  
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[2 4]), ...
                QP{qn}.motor(2,[2 4]), ...
                QP{qn}.motor(3,[2 4]), ...
                '-', 'Color', 'k', 'LineWidth',1);  
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[5 6]), ...
                QP{qn}.motor(2,[5 6]), ...
                QP{qn}.motor(3,[5 6]), ...
                'Color', QP{qn}.color, 'LineWidth', 2);

            hold(QP{qn}.h_3d, 'off')
        end

        % Run simulation
        x{qn}(1) =  local_x(iter,qn);
        x{qn}(2) =  local_y(iter,qn);
        x{qn}(3) =  local_z(iter,qn);
        x{qn}(4) =  local_vx(iter,qn);
        x{qn}(5) =  local_vy(iter,qn);
        x{qn}(6) =  local_vz(iter,qn);
        Rot0   = RPYtoRot_ZXY(local_roll(iter,qn), local_pitch(iter,qn), local_yaw(iter,qn));
        Quat0  = RotToQuat(Rot0);
        x{qn}(7) =  Quat0(1);
        x{qn}(8) =  Quat0(2);
        x{qn}(9) =  Quat0(3);
        x{qn}(10) =  Quat0(4);
        
        if mod(iter,50) == 0 
            hold(QP{qn}.h_3d, 'on')
            plot3(QP{qn}.h_3d, QP{qn}.state(1), QP{qn}.state(2), QP{qn}.state(3), 'r.');
            plot3(QP{qn}.h_3d, QP{qn}.des_state(1), QP{qn}.des_state(2), QP{qn}.des_state(3), 'r.');
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[1 2]), ...
                QP{qn}.motor(2,[1 2]), ...
                QP{qn}.motor(3,[1 2]), ...
                'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 5);   
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[3 4]), ...
                QP{qn}.motor(2,[3 4]), ...
                QP{qn}.motor(3,[3 4]), ...
                'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[1 3]), ...
                QP{qn}.motor(2,[1 3]), ...
                QP{qn}.motor(3,[1 3]), ...
                '-', 'Color', 'k', 'LineWidth',1);  
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[2 4]), ...
                QP{qn}.motor(2,[2 4]), ...
                QP{qn}.motor(3,[2 4]), ...
                '-', 'Color', 'k', 'LineWidth',1);  
            plot3(QP{qn}.h_3d, ...
                QP{qn}.motor(1,[5 6]), ...
                QP{qn}.motor(2,[5 6]), ...
                QP{qn}.motor(3,[5 6]), ...
                'Color', QP{qn}.color, 'LineWidth', 2);
            hold(QP{qn}.h_3d, 'off')
        end
        

        % Update quad plot
        if iter < max_iter
            desired_state.pos = [local_x(iter+1,qn);local_y(iter+1,qn);local_z(iter+1,qn)];
            desired_state.vel = [local_vx(iter+1,qn);local_vy(iter+1,qn);local_vz(iter+1,qn)];
            desired_state.acc = [0;0;0];
            desired_state.yaw = local_yaw(iter+1,qn);
            desired_state.yawdot = 0;
        end
        QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time + cstep);
        if OUTPUT_TO_VIDEO == 1
            %im = frame2im(getframe(gcf));
            im = frame2im(getframe(app.fig_3d));
            writeVideo(v,im);
        end
    end
    time = time + cstep; % Update simulation time
end

if OUTPUT_TO_VIDEO == 1
    close(v);
end


fprintf('finished.\n')