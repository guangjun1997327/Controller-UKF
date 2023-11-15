%% Transform Data with GPS into Timeseries for Sensorfusion Tests
%
% load data to transform it

data = load("C:\Users\laura\Documents\Greenteam\DV\98_Testdaten_E10\AutoXSiRiFSS.mat");
names = fieldnames(data);
timeStep = 0.01; %second, set desired time step


signalnames_desired = {...
    't';...
    'ms_ad_pedal_value_steering_angle';...
    'ms_ad_pedal_value_torque';...
    'ms_ad_pedal_value_brake_rec';...
    'ms_ad_pedal_value_brake_mech';...
    'ms_9dgps_front_angular_rate_x';...
    'ms_9dgps_front_angular_rate_y';...
    'ms_9dgps_front_angular_rate_z';...
    'ms_9dgps_front_angle_alpha';...
    'ms_9dgps_front_angle_beta';...
    'ms_9dgps_front_angle_gamma';...
    'ms_9dgps_front_accel_x';...
    'ms_9dgps_front_accel_y';...
    'ms_9dgps_front_accel_z';...
    'ms_9dgps_front_compass_x';...
    'ms_9dgps_front_compass_y';...
    'ms_9dgps_front_compass_z';...
    'ms_9dgps_front_state_cpu_temp';...
    'ms_9dgps_front_GPS_Latitude';...
    'ms_9dgps_front_GPS_Longitude';...
    'ms_9dgps_rear_accel_x';...
    'ms_9dgps_rear_accel_y';...
    'ms_9dgps_rear_accel_z';...
    'ms_9dgps_rear_angular_rate_x';...
    'ms_9dgps_rear_angular_rate_y';...
    'ms_9dgps_rear_angular_rate_z';...
    'ms_9dgps_rear_angle_alpha';...
    'ms_9dgps_rear_angle_beta';...
    'ms_9dgps_rear_angle_gamma';...
    'ms_9dgps_rear_compass_x';...
    'ms_9dgps_rear_compass_y';...
    'ms_9dgps_rear_compass_z';...
    'ms_9dgps_rear_state_cpu_temp';...
    'ms_v2o_data2_speed_longitudinal';...
    'ms_v2o_data2_speed_lateral';...
    'ms_v2o_data2_slip_angle';...
    'ms_ad_front_fds_right';...
    'ms_ad_front_fds_left';...
    'ms_ad_rear_fds_right';...
    'ms_ad_rear_fds_left';...
    'vd_ecu_motor_speed_can_fl';...
    'vd_ecu_motor_speed_can_fr';...
    'vd_ecu_motor_speed_can_rl';...
    'vd_ecu_motor_speed_can_rr'};

%% generate timeseries data
data_ts = {};
for n=2:length(signalnames_desired)
    try
        data_ts.(signalnames_desired{n}) = timeseries(data.(signalnames_desired{n}),data.t);
    catch
        warning(['Problem using generating timeseries for the Signal >' signalnames_desired{n} '<!!!'] )
    end
end
%% resample data
%get start and end times
% signalnames = fieldnames(data_ts);
% signalnames_desired = signalnames_desired(2:numel(signalnames_desired));
signalnames_desired_ = fieldnames(data_ts);
signalnum_desired = numel(signalnames_desired_);
t0 = data.t(1);
tN = data.t(end);

n_timeStep = (tN-t0)/(timeStep);
t_resampled = [0:timeStep:(timeStep*n_timeStep)];

for i=1:signalnum_desired
    try
        tsin = timeseries(data_ts.(signalnames_desired_{i}).Data, data.t-t0);
        signal_resampled = resample(tsin, t_resampled);
        data_resampled.(signalnames_desired_{i}) = signal_resampled.Data;
    catch 
        warning(['Problem using resampling time for the Signal >' signalnames_desired_{i} '<!!!'] )
    end
end
% for i=1:signalnum
%     try
%         signaltimes.(signalnames{i}) = data_ts.(signalnames{i}).Time;
%         signaltime = signaltimes.(signalnames{i}) - t0;
%         tsin = timeseries(data_ts.(signalnames{i}).Data, signaltime);
%         signal_resampled = resample(tsin, t_resampled);
%         data_resampled.(signalnames{i}) = signal_resampled.Data;
%     catch 
%         warning(['Problem using resampling time for the Signal >' signalnames{i} '<!!!'] )
%     end
% end

data_resampled.t = t_resampled';

%% rename signal time series for vd_data_evaluation

t = data_resampled.t;

PT_MotorSpeed_FL           = timeseries(single(data_resampled.vd_ecu_motor_speed_can_fl),t); 
PT_MotorSpeed_FR           = timeseries(single(data_resampled.vd_ecu_motor_speed_can_fr),t); 
PT_MotorSpeed_RL           = timeseries(single(data_resampled.vd_ecu_motor_speed_can_rl),t); 
PT_MotorSpeed_RR           = timeseries(single(data_resampled.vd_ecu_motor_speed_can_rr),t); 

VD_Gyro_Front_ARate_X      = timeseries(single(data_resampled.ms_9dgps_front_angular_rate_x),t); 
VD_Gyro_Front_ARate_Y      = timeseries(single(data_resampled.ms_9dgps_front_angular_rate_y),t); 
VD_Gyro_Front_ARate_Z      = timeseries(single(data_resampled.ms_9dgps_front_angular_rate_z),t); 

VD_Gyro_Front_Accel_X      = timeseries(single(data_resampled.ms_9dgps_front_accel_x),t); 
VD_Gyro_Front_Accel_Y      = timeseries(single(data_resampled.ms_9dgps_front_accel_y),t); 
VD_Gyro_Front_Accel_Z      = timeseries(single(data_resampled.ms_9dgps_front_accel_z),t); 

VD_Gyro_Front_Angle_X      = timeseries(single(data_resampled.ms_9dgps_front_angle_alpha),t); 
VD_Gyro_Front_Angle_Y      = timeseries(single(data_resampled.ms_9dgps_front_angle_beta),t); 
VD_Gyro_Front_Angle_Z      = timeseries(single(data_resampled.ms_9dgps_front_angle_gamma),t); 

VD_Gyro_Front_Comp_X       = timeseries(single(data_resampled.ms_9dgps_front_compass_x),t); 
VD_Gyro_Front_Comp_Y       = timeseries(single(data_resampled.ms_9dgps_front_compass_y),t); 
VD_Gyro_Front_Comp_Z       = timeseries(single(data_resampled.ms_9dgps_front_compass_z),t); 
VD_Gyro_Front_Temp         = timeseries(single(data_resampled.ms_9dgps_front_state_cpu_temp),t); 

VD_Gyro_Front_GPS_Latitude = timeseries(single(data_resampled.ms_9dgps_front_GPS_Latitude),t);
VD_Gyro_Front_GPS_Longitude = timeseries(single(data_resampled.ms_9dgps_front_GPS_Longitude),t);


VD_Gyro_Rear_ARate_X       = timeseries(single(data_resampled.ms_9dgps_rear_angular_rate_x),t); 
VD_Gyro_Rear_ARate_Y       = timeseries(single(data_resampled.ms_9dgps_rear_angular_rate_y),t); 
VD_Gyro_Rear_ARate_Z       = timeseries(single(data_resampled.ms_9dgps_rear_angular_rate_z),t); 

VD_Gyro_Rear_Accel_X       = timeseries(single(data_resampled.ms_9dgps_rear_accel_x),t); 
VD_Gyro_Rear_Accel_Y       = timeseries(single(data_resampled.ms_9dgps_rear_accel_y),t); 
VD_Gyro_Rear_Accel_Z       = timeseries(single(data_resampled.ms_9dgps_rear_accel_z),t); 

VD_Gyro_Rear_Angle_X       = timeseries(single(data_resampled.ms_9dgps_rear_angle_alpha),t); 
VD_Gyro_Rear_Angle_Y       = timeseries(single(data_resampled.ms_9dgps_rear_angle_beta),t); 
VD_Gyro_Rear_Angle_Z       = timeseries(single(data_resampled.ms_9dgps_rear_angle_gamma),t); 

VD_Gyro_Rear_Comp_X        = timeseries(single(data_resampled.ms_9dgps_rear_compass_x),t); 
VD_Gyro_Rear_Comp_Y        = timeseries(single(data_resampled.ms_9dgps_rear_compass_y),t); 
VD_Gyro_Rear_Comp_Z        = timeseries(single(data_resampled.ms_9dgps_rear_compass_z),t); 
VD_Gyro_Rear_Temp          = timeseries(single(data_resampled.ms_9dgps_rear_state_cpu_temp),t); 

VD_SteeringWheelAngle      = timeseries(single(data_resampled.ms_ad_pedal_value_steering_angle),t); 
VD_Suspension_Hall_FL       = timeseries(single(data_resampled.ms_ad_front_fds_left),t); 
VD_Suspension_Hall_FR       = timeseries(single(data_resampled.ms_ad_front_fds_right),t); 
VD_Suspension_Hall_RL       = timeseries(single(data_resampled.ms_ad_rear_fds_left),t); 
VD_Suspension_Hall_RR       = timeseries(single(data_resampled.ms_ad_rear_fds_right),t); 
 
VD_V2O_SpeedLateral        = timeseries(single(data_resampled.ms_v2o_data2_speed_lateral),t); 
VD_V2O_SpeedLongitudinal   = timeseries(single(data_resampled.ms_v2o_data2_speed_longitudinal),t); 
VD_V2O_SlipAngle           = timeseries(single(data_resampled.ms_v2o_data2_slip_angle),t);

VD_pedal_value_brake_mech  = timeseries(single(data_resampled.ms_ad_pedal_value_brake_mech),t); 
VD_pedal_value_brake_rec   = timeseries(single(data_resampled.ms_ad_pedal_value_brake_rec),t); 
VD_pedal_value_torque      = timeseries(single(data_resampled.ms_ad_pedal_value_torque),t); 

save('AutoXSiRiFSS_mitslipGPS_timeseries.mat', 't', 'PT_MotorSpeed_FL', 'PT_MotorSpeed_FR', 'PT_MotorSpeed_RL', 'PT_MotorSpeed_RR', 'VD_Gyro_Front_ARate_X', 'VD_Gyro_Front_ARate_Y', 'VD_Gyro_Front_ARate_Z', 'VD_Gyro_Front_Accel_X', 'VD_Gyro_Front_Accel_Y', 'VD_Gyro_Front_Accel_Z', 'VD_Gyro_Front_Angle_X', 'VD_Gyro_Front_Angle_Y', 'VD_Gyro_Front_Angle_Z', 'VD_Gyro_Front_Comp_X', 'VD_Gyro_Front_Comp_Y', 'VD_Gyro_Front_Comp_Z', 'VD_Gyro_Front_Temp', 'VD_Gyro_Front_GPS_Latitude', 'VD_Gyro_Front_GPS_Longitude', 'VD_Gyro_Rear_ARate_X', 'VD_Gyro_Rear_ARate_Y', 'VD_Gyro_Rear_ARate_Z', 'VD_Gyro_Rear_Accel_X', 'VD_Gyro_Rear_Accel_Y', 'VD_Gyro_Rear_Accel_Z', 'VD_Gyro_Rear_Angle_X', 'VD_Gyro_Rear_Angle_Y', 'VD_Gyro_Rear_Angle_Z', 'VD_Gyro_Rear_Comp_X', 'VD_Gyro_Rear_Comp_Y', 'VD_Gyro_Rear_Comp_Z', 'VD_Gyro_Rear_Temp', 'VD_SteeringWheelAngle', 'VD_Suspension_Hall_FL', 'VD_Suspension_Hall_FR', 'VD_Suspension_Hall_RL', 'VD_Suspension_Hall_RR', 'VD_V2O_SpeedLateral', 'VD_V2O_SpeedLongitudinal', 'VD_V2O_SlipAngle', 'VD_pedal_value_brake_mech', 'VD_pedal_value_brake_rec', 'VD_pedal_value_torque')


