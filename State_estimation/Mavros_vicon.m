% Load the Data
pose = readtable('pose.csv');
vicon = readtable('vicon.csv');

% Variables
d = 10^9;
new_eular_pose = [];

% Mavros and Vicon Data
sec_pose = pose(:,3);
secn_pose = pose(:,4);
secn_pose = secn_pose./d;


sec_vicon = vicon(:,3);
secn_vicon = vicon(:,4);
secn_vicon = secn_vicon./d;

secn_pose = table2array(secn_pose);
sec_pose = table2array(sec_pose);
secn_vicon = table2array(secn_vicon);
sec_vicon = table2array(sec_vicon);


time_pose = sec_pose+secn_pose;
time_vicon = sec_vicon+secn_vicon;

Min_time = min(time_pose);




time_pose = time_pose - Min_time;
time_vicon = time_vicon - Min_time;

pose_x_q = pose(:,9);
pose_y_q = pose(:,10);
pose_z_q = pose(:,11);
pose_w_q = pose(:,12);

Quat_pose = [pose_w_q,pose_x_q,pose_y_q, pose_z_q];
Quat_pose = table2array(Quat_pose);
eular_pose = quat2eul(Quat_pose,"ZYX");
eular_pose = rad2deg(eular_pose);




vicon_x_q = vicon(:,10);
vicon_y_q = vicon(:,11);
vicon_z_q = vicon(:,12);
vicon_w_q = vicon(:,13);

Quat_vicon = [vicon_w_q,vicon_x_q,vicon_y_q, vicon_z_q];
Quat_vicon = table2array(Quat_vicon);
eular_vicon = quat2eul(Quat_vicon,"ZYX");
eular_vicon = rad2deg(eular_vicon);

eular_pose= flip(eular_pose, 2);
eular_vicon = flip(eular_vicon, 2);

for c = 1:l_p
    R = rotz(-90);
    y = eular_pose(c,:);
    x = y*R;
    new_eular_pose =[new_eular_pose;x];
end


% Plotting the data
tiledlayout(3,1)

nexttile
plot(time_vicon,eular_vicon(:,1));
hold on
plot(time_pose,new_eular_pose(:,1));
hold off
xlabel('Time')
ylabel('Angle')
legend('MAVROS', 'VICON');

nexttile
plot(time_vicon,eular_vicon(:,2));
hold on
plot(time_pose,new_eular_pose(:,2));
hold off
xlabel('Time')
ylabel('Angle')
legend('MAVROS', 'VICON');

nexttile
plot(time_vicon,eular_vicon(:,3));
hold on
plot(time_pose,new_eular_pose(:,3));
hold off
xlabel('Time')
ylabel('Angle')
legend('MAVROS', 'VICON');



