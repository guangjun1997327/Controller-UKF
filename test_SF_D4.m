%% Test & Validation Script for test_com_SensorFusion_SF_D4
% Before you run this make sure that the testscripts are in your working
% directory, either with executing the script controller_init or doing it by
% hand
%% Load Data
% Insert testdata from the scenario you want to test the sensorfusion for here
% Data from Accel with named outliers
load('../../98_Testdaten_E10/FSS_Accel_Franzi_timeseries.mat');
%load('../../98_Testdaten_E10/FSS_Accel_Franzi_ts_outlier_vx.mat');
%load('../../98_Testdaten_E10/FSS_Accel_Franzi_ts_outlier_vy.mat');
%load('../../98_Testdaten_E10/FSS_Accel_Franzi_ts_outlier_v.mat');
% Data from Skidpad with named outliers
%load('../../98_Testdaten_E10/FSG_SkipPad_Siri_timeseries.mat');
%load('../../98_Testdaten_E10/FSG_SkidPad_SiRi_ts_outlier_vx.mat');
%load('../../98_Testdaten_E10/FSG_SkidPad_SiRi_ts_outlier_vy.mat');
% Data without constructed outliers
%load('../../98_Testdaten_E10/FSA_Skidpad_ALde_timeseries.mat');
%load('../../98_Testdaten_E10/FSS_Accel_NiMue_timeseries.mat');
%load('../../98_Testdaten_E10/FSS_AUtoX_MaWue_timeseries.mat');
%load('../../98_Testdaten_E10/FSG_AUtoX_Alde_timeseries.mat');
%load('../../98_Testdaten_E10/FSS_Endurance_timeseries.mat');
%load('../../98_Testdaten_E10/FSG_Endurance_timeseries.mat');
%load('../../98_Testdaten_E10/2020-07-05--16-19-34_PaSch.mat')

% With GPS data -> enable GPS as parameter?
%load('../../98_Testdaten_E10/FSG_timeseries.mat');
%load('../../98_Testdaten_E10/FSS_timeseries.mat');
%load('../../98_Testdaten_E10/FSEast_timeseries.mat');

%% Run simulation
% get length of data -> determine length of simulation
stop = PT_MotorSpeed_FL.Time(end);

sim('tests/test_com_SensorFusion_SF_D4_ukf.slx')
set_param('test_com_SensorFusion_SF_D4_ukf','StopTime','stop')

% mit GPS
% sim('tests/test_com_SensorFusion_SF_D4_slip_GPS_ukf.slx')
% set_param('test_com_SensorFusion_SF_D4_slip_GPS_ukf','StopTime','stop')


%% Compare results
% Position in x-direction
figure(1)
grid on
sgtitle('Vehicle Position and Error Estimation E10')
subplot(3,1,1)
plot(control_VehicleStates_result.PosX.time, control_VehicleStates_result.PosX.data);
hold on
grid on
plot(control_VehicleState_E10.PosX.time, control_VehicleState_E10.PosX.data);
% plot(VD_Gyro_Front_GPS_Longitude.time, (VD_Gyro_Front_GPS_Longitude.data-8.5).*10000)
title('Position in x')
xlabel('Time in s')
ylabel('Position in m')

subplot(3,1,2)
plot(control_VehicleStates_result.PosY.time, control_VehicleStates_result.PosY.data);
hold on
grid on
plot(control_VehicleState_E10.PosY.time, control_VehicleState_E10.PosY.data);
title('Position in y')
xlabel('Time in s')
ylabel('Position in m')

subplot(3,1,3)
plot(PosZ_eval.time, PosZ_eval.data);
hold on
grid on
% plot(PosZ_kalman.time, PosZ_kalman.data);
title('Position in z')
xlabel('Time in s')
ylabel('Position in m')

figure(2) % TBD transform Vel also in global coordinates
sgtitle('Vehicle Velocity Estimation')
subplot(2,1,1)
plot(control_VehicleStates_result.VelX.time,control_VehicleStates_result.VelX.data)
hold on
grid on
plot(sensor_signals.v2oVelocity.VelX.Time,sensor_signals.v2oVelocity.VelX.data)
%plot(control_VehicleState_E10.VelX.Time, control_VehicleState_E10.VelX.data)
title('Vehicle Velocity lateral')
xlabel('Time in s')
ylabel('Velocity x in m/s')

subplot(2,1,2)
plot(control_VehicleStates_result.VelY.Time,control_VehicleStates_result.VelY.data)
hold on
grid on
plot(sensor_signals.v2oVelocity.VelY.Time,sensor_signals.v2oVelocity.VelY.data)
%plot(control_VehicleState_E10.VelY.Time, control_VehicleState_E10.VelY.data)
title('Vehicle Velocity longitudinal')
xlabel('Time in s')
ylabel('Velocity y in m/s')


figure(3)
subplot(3,1,1)
plot(control_VehicleStates_result.YawAngle.Time,control_VehicleStates_result.YawAngle.data)
hold on
plot(control_VehicleState_E10.YawAngle.Time, control_VehicleState_E10.YawAngle.data)
title('Yaw Angle')
xlabel('Time in s')
ylabel('Angle in rad')
subplot(3,1,2)
plot(rollangle_sf.Time,rollangle_sf.data)
hold on
%plot(E10_rolling.Time, E10_rolling.data)
title('E10 RollAngle Estimation')
xlabel('Time in s')
ylabel('Error in rad/s')
subplot(3,1,3)
plot(pitchangle_sf.Time,pitchangle_sf.data)
hold on
%plot(E10_pitching.Time, E10_pitching.data)
title('E10 PitchAngle Estimation')
xlabel('Time in s')
ylabel('Error in rad/s')

figure(4)
plot(control_VehicleStates_result.PosX.data, control_VehicleStates_result.PosY.data);
hold on
grid on
plot(control_VehicleState_E10.PosX.data, control_VehicleState_E10.PosY.data);
title('Position')
xlabel('Position in x')
ylabel('Position in y')

figure(5)
subplot(3,1,1)
plot(C_tl_eval.time, C_tl_eval.data)
hold on
grid on
%plot(C_tl_E10.time, C_tl_E10.data)
title('TireLoad C')
xlabel('Time in s')
ylabel('C_tl in whatever')

subplot(3,1,2)
plot(D_tl_eval.time, D_tl_eval.data)
hold on
grid on
%plot(D_tl_E10.time, D_tl_E10.data)
title('TireLoad D')
xlabel('Time in s')
ylabel('D_tl in whatever')

subplot(3,1,3)
plot(TireLoad_eval.time, TireLoad_eval.data)
hold on
grid on
plot(TireLoad_E10.time,TireLoad_E10.data)
title('TireLoad')
xlabel('Time in s')
ylabel('TireLoad in whatever')

figure(6)
subplot(2,1,1)
plot(control_VehicleStates_result.VelX.time,control_VehicleStates_result.VelX.data)
hold on
grid on
%plot(sensor_signals.v2oVelocity.VelX.Time,sensor_signals.v2oVelocity.VelX.data)
plot(control_VehicleState_E10.VelX.Time, control_VehicleState_E10.VelX.data)
title('Vehicle Velocity lateral')
xlabel('Time in s')
ylabel('Velocity x in m/s')
subplot(2,1,2)
%plot(control_VehicleStates_result.YawAngle.Time,control_VehicleStates_result.YawAngle.data)
hold on 
grid on
%plot(control_VehicleState_E10.YawAngle.Time, control_VehicleState_E10.YawAngle.data)
plot(chi)
title('Yaw Angle')
xlabel('Time in s')
ylabel('Angle in rad')
