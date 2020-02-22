%% Project 3 - Vicknesh Balabaskaran
%%
clear; clc; close all;

%% Controller Parameter Definitions
run Geometry.m; % creates a linkList in the workspace for you and for the simulation
load points3D; % loads the CSM trajectory points

%% Define Kp and Kd gains
Kp = 1000;
Kd = 700;
th_last=[0;-pi/2;0;0;0;0];
[n,V,p] = affine_fit(points3D);
t=[0 2 2.5:0.35:9.5 12];
points=points3D;
transPercent=0.025;

%% Part A

% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

% Enable/Disable Control
control_enable = true;
set_param('Project3_System_partA/control_enable', 'sw', int2str(control_enable))

% Run Simulation
simTime = 12;
simOut =  sim('Project3_System_partA','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
position_error = simOut.get('position_error');

error=(position_error.signals.values(:,1:3))';
x_error=sqrt(((position_error.signals.values(:,1))).^2)';
y_error=sqrt(((position_error.signals.values(:,2))).^2)';
z_error=sqrt(((position_error.signals.values(:,3))).^2)';
pos_error=sqrt(mean(error.^2));
% end
% Plot theta as a function of time
figure
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
% 
% Display Arm Motion Movie
figure
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end
%% Part A - RMS Position Error
figure
plot(theta_actual.time,pos_error)
title('RMS Position error')
xlabel(['RMS_{error}', '(s)'])
ylabel('time (t)')

%% Part A - X Error
figure
plot(theta_actual.time,x_error')
title('X_{error}')
xlabel(['X_{error}', '(s)'])
ylabel('time (t)')

%% Part A - Y Error
figure
plot(theta_actual.time,y_error')
title('Y_{error}')
xlabel(['Y_{error}', '(s)'])
ylabel('time (t)')

%% Part A - Z Error
figure
plot(theta_actual.time,z_error')
title('Z_{error}')
xlabel(['Z_{error}', '(s)'])
ylabel('time (t)')


%% Part B
% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

% Enable/Disable Control
control_enable = true;
set_param('Project3_System_partB/control_enable', 'sw', int2str(control_enable))

% Run Simulation
simTime = 10;
simOut =  sim('Project3_System_partB','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
trans_error = simOut.get('trans_error');

error=(trans_error.signals.values(:,1:3))';
x_error=sqrt(((trans_error.signals.values(:,1))).^2)';
y_error=sqrt(((trans_error.signals.values(:,2))).^2)';
z_error=sqrt(((trans_error.signals.values(:,3))).^2)';
pos_error=sqrt(mean(error.^2));
% % end
% % Plot theta as a function of time
figure
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end
% 
% % Display Arm Motion Movie
figure
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end
% % 
%% Part B - RMS Position Error
figure
plot(theta_actual.time,pos_error)
title('RMS Position error')
xlabel(['RMS_{error}', '(s)'])
ylabel('time (t)')

%% Part B - X Error
figure
plot(theta_actual.time,x_error')
title('X_{error}')
xlabel(['X_{error}', '(s)'])
ylabel('time (t)')

%% Part B - Y Error
figure
plot(theta_actual.time,y_error')
title('Y_{error}')
xlabel(['Y_{error}', '(s)'])
ylabel('time (t)')

%% Part B - Z Error
figure
plot(theta_actual.time,z_error')
title('Z_{error}')
xlabel(['Z_{error}', '(s)'])
ylabel('time (t)')



%% Part C
% Choose Simulation System (perfect model or realistic model)
Sim_Exact = true; % sets if the simulated geometry is exact (true) or slightly different (false)

% Enable/Disable Control
control_enable = true;
set_param('Project3_System_partC/control_enable', 'sw', int2str(control_enable))

% Run Simulation
simTime = 10;
simOut =  sim('Project3_System_partC','SimulationMode','normal','AbsTol','1e-5','StopTime', int2str(simTime),...
    'SaveState','on','StateSaveName','xout',...
    'SaveOutput','on','OutputSaveName','yout',...
    'SaveFormat', 'array');


% Extract Variables From Simulation
laser_tracking = simOut.get('laser_tracking');
theta_dot_actual = simOut.get('theta_actual');
theta_actual = simOut.get('theta_actual');
control_torque = simOut.get('control_torque');
trans_error = simOut.get('trans_error');

error=(trans_error.signals.values(:,1:3))';
x_error=sqrt(((trans_error.signals.values(:,1))).^2)';
y_error=sqrt(((trans_error.signals.values(:,2))).^2)';
z_error=sqrt(((trans_error.signals.values(:,3))).^2)';
pos_error=sqrt(mean(error.^2));
% end
% Plot theta as a function of time
figure
for i=1:6
    subplot(3,2,i)
    plot(theta_actual.time,theta_actual.signals.values(:,i))
    title(['\theta_', int2str(i)])
    xlabel('time (s)')
    ylabel('angle (rad)')
    grid on;
end

% Display Arm Motion Movie
figure
plot(0,0); ah = gca; % just need a current axis handel
stepSize = fix((1/30)/theta_actual.time(2)); % 30 frames per second
for i=1:stepSize:length(theta_actual.time)
   plotArm(theta_actual.signals.values(i,:),Sim_Exact,ah);
   hold on;
   plot3(reshape(laser_tracking.signals.values(1,4,1:i),[i,1]),... % x pos
         reshape(laser_tracking.signals.values(2,4,1:i),[i,1]),... % y pos
         reshape(laser_tracking.signals.values(3,4,1:i),[i,1]),'r','Parent',ah); % z pos
   plot3(points3D(:,1),points3D(:,2),points3D(:,3),'m','Parent',ah);
   hold off;
   pause(1/30)
end

%% Part C - RMS Position Error
figure
plot(theta_actual.time,pos_error)
title('RMS Position error')
xlabel(['RMS_{error}', '(s)'])
ylabel('time (t)')

% Part C - X Error
figure
plot(theta_actual.time,x_error')
title('X_{error}')
xlabel(['X_{error}', '(s)'])
ylabel('time (t)')

% Part C - Y Error
figure
plot(theta_actual.time,y_error')
title('Y_{error}')
xlabel(['Y_{error}', '(s)'])
ylabel('time (t)')

% Part C - Z Error
figure
plot(theta_actual.time,z_error')
title('Z_{error}')
xlabel(['Z_{error}', '(s)'])
ylabel('time (t)')