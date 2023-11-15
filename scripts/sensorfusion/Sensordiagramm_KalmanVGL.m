Stopzeit=125;
r_dyn=0.221;

%% YawRate Sensor vergleich zu Kalmanfilter 1
figure

subplot(1,4,1)
plot(VD_Gyro_Front_ARate_Z/(180/pi));
axis([0,Stopzeit,-2,2]);
title('Sensor 9D front YawRate');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;
subplot(1,4,2)
plot(VD_Gyro_Rear_ARate_Z/(180/pi));
hold on;
plot(YawRate_kalman);
legend('Sensor rear', 'Kalman')
axis([0,Stopzeit,-2,2]);
title('Sensor 9D rear YawRate');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;
subplot(1,4,3)
plot(YawRate_kalman);
axis([0,Stopzeit,-2,2]);
title('YawRate from Kalman');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;
subplot(1,4,4)
plot(AccY_kalman/VelX_kalman);
axis([0,Stopzeit,-2,2]);
title('YawRate aus AccY/VelX');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;
%% AccY 2
figure

subplot(1,4,1)
plot(AccY_kalman);
axis([0,Stopzeit,-15,15]);
title('AccY');
xlabel('Time in sec');
ylabel('AccY in m/sec^2');
grid on;
subplot(1,4,2)
plot(AccY_kalman/VelX_kalman);
axis([0,Stopzeit,-2,2]);
title('YawRate aus AccY/VelX');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;

subplot(1,4,3)
plot(VD_Gyro_Front_ARate_Y*9.81);
hold on;
plot(AccY_kalman, 'color','black');
axis([0,Stopzeit,-15,15]);
title('AccY kalman und Sensor front');
xlabel('Time in sec');
ylabel('Acc in m/sec^2');
grid on;

subplot(1,4,4)
plot(VD_Gyro_Rear_ARate_Y*9.81);
hold on;
plot(AccY_kalman, 'color','black');
axis([0,Stopzeit,-15,15]);
title('AccY kalman und Sensor rear');
xlabel('Time in sec');
ylabel('Acc in m/sec^2');
grid on;
%% VelX Schwimmwinkel  3
figure

subplot(1,3,1)
plot(VD_V2O_SpeedLongitudinal);
hold on; 
plot(VelX_kalman);

legend('Schwimmwinkel', 'Sensorfusion')
axis([0,Stopzeit,0,14]);
title('Sensor SlipAngle v2o Sensor');
xlabel('Time in sec');
ylabel('VelX in m/sec');
grid on;

subplot(1,3,2)
plot(VelX_kalman);
axis([0,Stopzeit,0,15]);
title('VelX from Kalmanfilter');
xlabel('Time in sec');
ylabel('VelX in m/sec');

grid on;

subplot(1,3,3)
plot(VelX_kalman);
axis([0,Stopzeit,0,15]);
title('VelX Ersatzrechnung');
xlabel('Time in sec');
ylabel('VelX in m/sec');
grid on;

%%AccX 4

figure
subplot(1,2,1)

plot(AccX_Rear);
hold on;
plot(AccX_Front)
axis([0,Stopzeit,-10,10]);
legend( 'AccX rear', 'AccX front');
title('AccX');
xlabel('Time in sec');
ylabel('AccX in m/sec^2');
grid on;

subplot(1,2,2)
plot(AccX_kalman, 'color','blue');
axis([0,Stopzeit,-10,10]);
title('AccX kalman, Durchschnitt aus Sensoren');
legend('AccX kalman');
xlabel('Time in sec');
ylabel('AccX in m/sec^2');
grid on;
%% YawRate aus alternativ Rechnung  5
figure
subplot(1,2,1)
plot(VD_V2O_SpeedLateral);
hold on; 
plot(VelY_kalman);

legend('Schwimmwinkel', 'Sensorfusion Schwerpunkt')
axis([0,Stopzeit,-3,3]);
title('VelY=VelYmeasurement-YawRate*Helbelarm');
xlabel('Time in sec');
ylabel('VelY in m/sec');
grid on;

subplot(1,2,2)
plot(YawRate_kalman);
axis([0,Stopzeit,-2,2]);
title('YawRate from Kalman');
xlabel('Time in sec');
ylabel('YawRate in rad/sec');
grid on;
hold off;

%% VelY auswertung 6

figure
% hold on;

plot(VelY_input);

axis([0,Stopzeit,-5,5]);
legend('VelY_input', 'Vely berechnet');
title('VelY Schwimmwinkelsensor');
xlabel('Time in sec');
ylabel('VelY in m/sec');
grid on;
hold off;



%% TireLoads

figure
% hold on;
subplot(5,1,1)

plot(TireLoad_FL, 'color','yellow');
legend('TireLoad_FL', 'TireLoad_FR', 'TireLoad_RL', 'TireLoad_RR');
hold on;
plot(TireLoad_FR, 'color','blue');
hold on;
plot(TireLoad_RL, 'color','red');


plot(TireLoad_RR, 'color','green');
legend('Front Left','Front Right', 'Rear Left', 'Rear Right')
axis([0,Stopzeit,0,1800]);
title('TireLoads');
xlabel('Time in sec');
ylabel('Force in Newton');
grid on;

subplot(5,1,2)
vehicle_mass=240;
sum_TireLoads=((TireLoad_FL+TireLoad_FR+TireLoad_RL+TireLoad_RR)/9.81)/vehicle_mass;
plot(sum_TireLoads);
axis([0,Stopzeit,0,3]);
title('Tireloads');
xlabel('Time in sec');
grid on;

subplot(5,1,3)

plot(RockerAngle_FL, 'color','yellow');
legend('RockerAngle_FL', 'RockerAngle_FR', 'RockerAngle_RL', 'RockerAngle_RR');
hold on;
plot(RockerAngle_FR, 'color','blue');
hold on;
plot(RockerAngle_RL, 'color','red');


plot(RockerAngle_RR, 'color','green');
legend('Front Left','Front Right', 'Rear Left', 'Rear Right')
axis([0,Stopzeit,-15,15]);
title('RockerAngle');
xlabel('Time in sec');
ylabel('Angle in rad');
grid on;

subplot(5,1,4)

plot(TireLoad_FL, 'color','yellow');
legend('RockerAngle_FL', 'RockerAngle_FR', 'RockerAngle_RL', 'RockerAngle_RR');
hold on;
plot(TireLoad_FR, 'color','blue');
hold on;
plot(TireLoad_RL, 'color','red');


plot(TireLoad_RR, 'color','green');
legend('Front Left','Front Right', 'Rear Left', 'Rear Right')
axis([0,Stopzeit,0,1800]);
title('TireLoads E8');
xlabel('Time in sec');
ylabel('Force in N');
grid on;


subplot(5,1,5)
vehicle_mass=240;
sum_TireLoads=((TireLoad_FL+TireLoad_FR+TireLoad_RL+TireLoad_RR)/9.81)/vehicle_mass;
plot(sum_TireLoads);
axis([0,Stopzeit,0,3]);
title('Tireloads');
xlabel('Time in sec');
grid on;
hold off;
%% Orientierung, nur ergebnisse aus kalman filter

figure
subplot(3,1,1)

plot(PosZ_kalman, 'color','red');
hold on;
plot(VelZ_kalman, 'color','blue');

legend('PosZ in m', 'VelZ in m/sec');
axis([0,Stopzeit,-0.005,0.005]);
title('Orientierung');
xlabel('Time in sec');
ylabel('Force in Newton');
grid on;

subplot(3,1,2)

plot(RollAngle_kalman, 'color','cyan');
hold on;
plot(RollRate_kalman, 'color','blue');

legend('RollAngle', 'RollRate');
axis([0,Stopzeit,-0.01,0.01]);
title('RollAngle und Rate');
xlabel('Time in sec');
% ylabel('Angle in rad');
grid on;

subplot(3,1,3)

plot(PitchAngle_kalman, 'color','green');
hold on;
plot(PitchRate_kalman, 'color','blue');

legend('PitchAngle','PitchRate')
axis([0,Stopzeit,-0.01,0.01]);
title('PitchAngle and Rate');
xlabel('Time in sec');
% ylabel('Angle in rad');
grid on;
hold off;

%% Schlüpfe, 

figure
% hold on;
subplot(4,1,1)

plot(TireSlipLong_FL, 'color','red');

legend('Output kalman', 'normale Berechnung');
axis([0,Stopzeit,-1,1]);
title('TireSlipLong_FL');
xlabel('Time in sec');
ylabel('Slip in Prozent');
grid on;
hold off;

subplot(4,1,2)

plot(TireSlipLong_FR, 'color','red');
legend('Output kalman', 'normale Berechnung');
axis([0,Stopzeit,-1,1]);
title('TireSlipLong_FR');
xlabel('Time in sec');
ylabel('Slip in Prozent');
grid on;
hold off;

subplot(4,1,3)

plot(TireSlipLong_RL, 'color','red');

legend('Output kalman', 'normale Berechnung');
axis([0,Stopzeit,-1,1]);
title('TireSlipLong_RL');
xlabel('Time in sec');
ylabel('Slip in Prozent');
grid on;
hold off;

subplot(4,1,4)

plot(TireSlipLong_RR, 'color','red');

legend('Output kalman', 'normale Berechnung');
axis([0,Stopzeit,-1,1]);
title('TireSlipLong_RR');
xlabel('Time in sec');
ylabel('Slip in Prozent');
grid on;
hold off;

%% WheelVelocity

figure
% hold on;
subplot(4,1,1)

plot(WheelVelocity_FL, 'color','red');
hold on;
plot(WheelSpeed_FL*r_dyn, 'color','blue');
hold on;
legend('Output kalman', 'measurement');
axis([0,Stopzeit,0,15]);
title('WheelVelocity FL');
xlabel('Time in sec');
ylabel('Velocity in m/sec');
grid on;
hold off;

subplot(4,1,2)

plot(WheelVelocity_FR, 'color','red');
hold on;
plot(WheelSpeed_FR*r_dyn, 'color','blue');

legend('Output kalman', 'measurement');
axis([0,Stopzeit,0,15]);
title('WheelVelocity Front Right');
xlabel('Time in sec');
ylabel('Velocity in m/sec');
grid on;

subplot(4,1,3)
plot(WheelVelocity_RL, 'color','red');
hold on;
plot(WheelSpeed_RL*r_dyn, 'color','blue');

legend('Output kalman', 'measurement');
axis([0,Stopzeit,0,15]);
title('WheelVelocity Rear Left ');
xlabel('Time in sec');
ylabel('Velocity in m/sec');
grid on;

subplot(4,1,4)

plot(WheelVelocity_RR, 'color','red');
hold on;
plot(WheelSpeed_RR*r_dyn, 'color','blue');

legend('Output kalman', 'measurement');
axis([0,Stopzeit,0,15]);
title('WheelVelocity Rear Right');
xlabel('Time in sec');
ylabel('Velocity in m/sec');
grid on;
hold off;

