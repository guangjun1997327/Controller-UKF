clear

[fn,pathname]=uigetfile();


ms = load([pathname,fn]);

fnames=fieldnames(ms);

try
VD_Gyro_Front_Accel_X = getTimeSeries('ms_9d_front_accel_x_',fnames,ms);
VD_Gyro_Front_Accel_Y = getTimeSeries('ms_9d_front_accel_y_',fnames,ms);
VD_Gyro_Front_Accel_Z = getTimeSeries('ms_9d_front_accel_z_',fnames,ms);
VD_Gyro_Front_ARate_X = getTimeSeries('ms_9d_front_angular_rate_x_',fnames,ms);
VD_Gyro_Front_ARate_Y = getTimeSeries('ms_9d_front_angular_rate_y_',fnames,ms);
VD_Gyro_Front_ARate_Z = getTimeSeries('ms_9d_front_angular_rate_z_',fnames,ms);
VD_Gyro_Front_Comp_X = getTimeSeries('ms_9d_front_compass_x_',fnames,ms);
VD_Gyro_Front_Comp_Y = getTimeSeries('ms_9d_front_compass_y_',fnames,ms);
VD_Gyro_Front_Comp_Z = getTimeSeries('ms_9d_front_compass_z_',fnames,ms);
VD_Gyro_Front_Temp = getTimeSeries('ms_9d_front_state_cpu_temp_',fnames,ms);
catch
    disp("Fehler beim einlesen des vorderen 9D Sensors");
end

try
VD_Gyro_Rear_Accel_X = getTimeSeries('ms_9d_rear_accel_x_',fnames,ms);
VD_Gyro_Rear_Accel_Y = getTimeSeries('ms_9d_rear_accel_y_',fnames,ms);
VD_Gyro_Rear_Accel_Z = getTimeSeries('ms_9d_rear_accel_z_',fnames,ms);
VD_Gyro_Rear_ARate_X = getTimeSeries('ms_9d_rear_angular_rate_x_',fnames,ms);
VD_Gyro_Rear_ARate_Y = getTimeSeries('ms_9d_rear_angular_rate_y_',fnames,ms);
VD_Gyro_Rear_ARate_Z = getTimeSeries('ms_9d_rear_angular_rate_z_',fnames,ms);
VD_Gyro_Rear_Comp_X = getTimeSeries('ms_9d_rear_compass_x_',fnames,ms);
VD_Gyro_Rear_Comp_Y = getTimeSeries('ms_9d_rear_compass_y_',fnames,ms);
VD_Gyro_Rear_Comp_Z = getTimeSeries('ms_9d_rear_compass_z_',fnames,ms);
VD_Gyro_Rear_Temp = getTimeSeries('ms_9d_rear_state_cpu_temp_',fnames,ms);
catch
    disp("Fehler beim einlesen des hinteren 9D Sensors");
end



try
VD_V2O_SpeedLongitudinal = getTimeSeries('ms_v2o_data2_speed_longitudinal_',fnames,ms);
VD_V2O_SpeedLateral = getTimeSeries('ms_v2o_data2_speed_lateral_',fnames,ms);
catch
    disp("Fehler beim einlesen des Schwimmwinkelsensors");
end

try
VD_pedal_value_torque = getTimeSeries('ms_ad_pedal_value_torque_',fnames,ms);
VD_pedal_value_brake_rec = getTimeSeries('ms_ad_pedal_value_brake_rec_',fnames,ms);
VD_pedal_value_brake_mech = getTimeSeries('ms_ad_pedal_value_brake_mech_',fnames,ms);%timeseries(boolean(ms.VD_Pedal_Brake_Mech),ms.t);
VD_SteeringWheelAngle = getTimeSeries('ms_ad_pedal_value_steering_angle_',fnames,ms);
catch
    disp("Fehler beim einlesen der Pedal Signale");
end

try
PT_MotorSpeed_FL = getTimeSeries('PT_MotorSpeed_FL_',fnames,ms);
PT_MotorSpeed_FR = getTimeSeries('PT_MotorSpeed_FR_',fnames,ms);
PT_MotorSpeed_RL = getTimeSeries('PT_MotorSpeed_RL_',fnames,ms);
PT_MotorSpeed_RR = getTimeSeries('PT_MotorSpeed_RR_',fnames,ms);
catch
    disp("Fehler beim einlesen der Motor Speeds");
end

try
VD_Suspension_Hall_FL = getTimeSeries('ms_ad_front_fds_left_',fnames,ms);
VD_Suspension_Hall_FR = getTimeSeries('ms_ad_front_fds_right_',fnames,ms);
VD_Suspension_Hall_RL = getTimeSeries('ms_ad_rear_fds_left_',fnames,ms);
VD_Suspension_Hall_RR = getTimeSeries('ms_ad_rear_fds_right_',fnames,ms);
catch
    disp("Fehler beim einlesen der Hall-Sensoren des FDS");
end

%% ab hier Signale, die eig nicht gebraucht werden

try
PT_TorqueCurrent_FL = getTimeSeries('PT_TorqueCurrent_FL_',fnames,ms);
PT_TorqueCurrent_FR = getTimeSeries('PT_TorqueCurrent_FR_',fnames,ms);
PT_TorqueCurrent_RL = getTimeSeries('PT_TorqueCurrent_RL_',fnames,ms);
PT_TorqueCurrent_RR = getTimeSeries('PT_TorqueCurrent_RR_',fnames,ms);
end

try
HV_pmeter_Voltage = getTimeSeries('HV_pmeter_Voltage_',fnames,ms);
HV_pmeter_Current = getTimeSeries('HV_pmeter_Current_',fnames,ms);
HV_pmeter_Energy = getTimeSeries('HV_pmeter_Energy_',fnames,ms);
end

try
HV_Accu_Cells_SoC = getTimeSeries('HV_Accu_Cells_SoC_',fnames,ms);
HV_Accu_Cells_MaxVoltage = getTimeSeries('HV_Accu_Cells_MaxVoltage_',fnames,ms);
HV_Accu_Cells_MinVoltage = getTimeSeries('HV_Accu_Cells_MinVoltage_',fnames,ms);
HV_Accu_Cells_AvgVoltage = getTimeSeries('HV_Accu_Cells_AvgVoltage_',fnames,ms);
HV_Accu_Cells_MinTemp = timeseries(uint8(ms.HV_Accu_Cells_MinTemp),ms.t);
end



try
HMI_DriverNotice = timeseries(logical(ms.HMI_DriverNotice),ms.t);
HMI_PushButton_6 = timeseries(logical(ms.HMI_PushButton_6),ms.t);
COM_Confirm = getTimeSeries('COM_Confirm_',fnames,ms);
end

plot(VD_V2O_SpeedLongitudinal);
%% Ab hier die Funktion auf der alles beruht

function ts = getTimeSeries(listname,fnames1,ms1)
    
    %fnames{find(contains(fnames,listname))}
    %strrep(fnames{find(contains(fnames,listname))},[listname '_'],'')
    ts=(timeseries(single(ms1.(fnames1{find(contains(fnames1,listname))})),ms1.(strrep(fnames1{find(contains(fnames1,listname))},listname,''))));
    %ts=fnames
end