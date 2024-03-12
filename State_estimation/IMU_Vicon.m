% Load the Data
data = readtable('imufilter.csv');
pose = readtable('pose.csv');

% Variables
d = 10^9;

% Acceleration & Gyroscope Data
sec_data = data(:,3);
secn_data = data(:,4);
secn_data = secn_data./d;


secn_data = table2array(secn_data);
sec_data = table2array(sec_data);

time_data = sec_data+secn_data;
M_data = min(time_data);
time_data = time_data - M_data;
time_data_min = min(time_data);
time_data_max = max(time_data);
le = length(time_data);
time_interval_data = linspace(time_data_min,time_data_max,le);


acc_x = data(:,15);
acc_y = data(:,16);
acc_z = data(:,17);

gyro_x = data(:,11);
gyro_y = data(:,12);
gyro_z = data(:,13);

ACC = [acc_x,acc_y,acc_z];
GYRO = [gyro_x,gyro_y,gyro_z];
ACC = table2array(ACC);
GYRO = table2array(GYRO);


% Mavros Data
pose_x_q = pose(:,9);
pose_y_q = pose(:,10);
pose_z_q = pose(:,11);
pose_w_q = pose(:,12);
sec_pose = pose(:,3);
secn_pose = pose(:,4);
secn_pose = secn_pose./d;
secn_pose = table2array(secn_pose);
sec_pose = table2array(sec_pose);
time_pose = sec_pose+secn_pose;


Quat_pose = [pose_w_q,pose_x_q,pose_y_q, pose_z_q];
Quat_pose = table2array(Quat_pose);
eular_pose = quat2eul(Quat_pose,"ZYX");
eular_pose = rad2deg(eular_pose);

M = min(time_pose);
time_pose = time_pose - M_pose;
Min_pose = min(time_pose);
Max_pose = max(time_pose);
pose_l = length(time_pose);
time_interval_pose = linspace(Min_pose,Max_pose,pose_l);

% IMUFilter
decim = 1;
Fs = 86.896;
fuse = imufilter('SampleRate',Fs,'DecimationFactor',decim);
q = fuse(ACC,GYRO);
q = quat2eul(q,'ZYX');
q = rad2deg(q);


% Plotting Data
tiledlayout(3,1)

nexttile
plot(time_interval_data,q(:,1))
hold on
plot(time_pose,eular_pose(:,1));
hold off
xlabel('Time')
ylabel('Angle')
legend('IMUFilter Estimation', 'MAVROS Estimation');

nexttile
plot(time_interval_data,q(:,2))
hold on
plot(time_pose,eular_pose(:,2));
hold off
xlabel('Time')
ylabel('Angle')
legend('IMUFilter Estimation', 'MAVROS Estimation');

nexttile
plot(time_interval_data,q(:,3))
hold on
plot(time_pose,eular_pose(:,3));
hold off
xlabel('Time')
ylabel('Angle')
legend('IMUFilter Estimation', 'MAVROS Estimation');

