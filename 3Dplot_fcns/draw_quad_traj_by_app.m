function draw_quad_traj_by_app( x, y, z, vx,vy,vz, roll ,pitch,yaw,isRecord,quadNum,app)
%x,y,z��λ�ã�NED����ϵ������Ƕ�ܴη�������λ�����ݣ���x(:,1)��ʾ��1�ܷ�������xλ�ã�x(:,2)��ʾ��2�ܷ�������xλ��...
%vx,vy,vz���ٶȣ�NED����ϵ
%roll,pitch,yaw����̬��NED����ϵ
%isRecord:�Ƿ��¼¼��=1��¼��=0����¼
%quadNum�����˻��ĸ���
%handles��GUI���ھ��

%��������ϵΪ��-��-�أ�����������ϵΪ��-��-��
local_yaw = pi/2-yaw;
local_pitch= -pitch;
local_roll = roll;
local_x = x;
local_y = y;
local_z = -z;  %ԭʼ����ϵΪNED����ͼʱZ������
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
%����3D��ͼ����
cla(app.fig_3d);
%��3D��ͼ����Ϊ��ǰ�����Ƶ�������
%axes(app.fig_3d);
% number of quadrotors
nquad = quadNum;

% max time
time_tol = size(local_x,1)*0.01;

%% **************************** FIGURES *****************************
fprintf('3D�����ʼ��...\n')
%h_fig = app.fig_3d;
%h_3d = gca;
%������GUI��������ת3D����
%rotate3d(app.fig_3d,"on");
axis(app.fig_3d,"equal");
grid(app.fig_3d, "on");
view(app.fig_3d,3);
xlabel(app.fig_3d,'x [m]'); 
ylabel(app.fig_3d,'y [m]'); 
zlabel(app.fig_3d,'z [m]')

%��ά��������
axis(app.fig_3d,[min_x*2-10,max_x*2+10,min_y*2-10,max_y*2+10,min_z,max_z*2+10])   
axis(app.fig_3d,"normal");
quadcolors = lines(nquad);

%set(gcf,'Renderer','OpenGL')
%% *********************** INITIAL CONDITIONS ***********************
fprintf('���ó�ʼ����...\n')
max_iter  = size(local_x,1);      % max iteration
starttime = 0;         % start of simulation in seconds
tstep     = 0.01;      % this determines the time step at which the solution is given
cstep     = 0.05;      % image capture time interval
time      = starttime; % current time

%׼��time=0ʱ������
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
%�Ƿ񱣴�Ϊ¼��
OUTPUT_TO_VIDEO = 0;
if isRecord == 1
    OUTPUT_TO_VIDEO = 1;
end
if OUTPUT_TO_VIDEO == 1
    v = VideoWriter('.\quad_traj.avi');
    open(v)
end

fprintf('���з���....')
% Main loop
for iter = 1:max_iter
    
    %time:��ǰʱ��
    timeint = time:tstep:time+cstep;

    %tic;
    
    %�Ǳ�ֻ��ʾ��1�����������Ϣ
    app.speed_meter.Airspeed= sqrt(local_vx(iter,1)^2+local_vy(iter,1)^2+local_vz(iter,1)^2)*3.6;  %km/h
    app.horizon_meter.Pitch= rad2deg(local_pitch(iter,1));  %deg
    app.horizon_meter.Roll= rad2deg(local_roll(iter,1));    %deg
    app.heading_meter.Heading=rad2deg(local_yaw(iter,1));   %deg
    
    
    % Iterate over each quad
    for qn = 1:nquad
        % Initialize quad plot
        if iter == 1
            %���Ƶ�һ�����ݡ�QuadPlot��һ����
            QP{qn} = QuadPlot(qn, x0{qn}, 1, 0.1, 'r', max_iter, app.fig_3d);
            %׼����2֡����
            desired_state.pos = [local_x(2,qn);local_y(2,qn);local_z(2,qn)];
            desired_state.vel = [local_vx(2,qn);local_vy(2,qn);local_vz(2,qn)];
            desired_state.acc = [0;0;0];
            desired_state.yaw = local_yaw(2,qn);
            desired_state.yawdot = 0;
            QP{qn}.UpdateQuadPlot(x{qn}, [desired_state.pos; desired_state.vel], time);
           
        end
        %���Ƶ�2������
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