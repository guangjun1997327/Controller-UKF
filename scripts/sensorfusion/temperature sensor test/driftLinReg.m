clear all
close all

%%      Skript zum berechnen des Temperaturdrifts des 9D Sensors

measurementName = '2016-11-12_TempDrift2_18-12-50';

load(measurementName)

Ts = 0.002;

startTime = 5;
endTime =  350;

%% Signale kürzen auf den gewünschten Zeitabschnitt und um 0 zentrieren (damit keine konstante Abweichung geschätzt wird)

signals(:,1) = Time(startTime/Ts+1:endTime/Ts+1);
signals(:,2) = detrend(VD_Gyro_Front_Temp(startTime/Ts+1:endTime/Ts+1),'constant');

signals(:,3) = detrend(VD_Gyro_Front_Accel_X(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,4) = detrend(VD_Gyro_Front_Accel_Y(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,5) = detrend(VD_Gyro_Front_Accel_Z(startTime/Ts+1:endTime/Ts+1,1),'constant');

signals(:,6) = detrend(VD_Gyro_Front_ARate_X(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,7) = detrend(VD_Gyro_Front_ARate_Y(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,8) = detrend(VD_Gyro_Front_ARate_Z(startTime/Ts+1:endTime/Ts+1,1),'constant');

signals(:,9) = detrend(VD_Gyro_Front_Comp_X(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,10) = detrend(VD_Gyro_Front_Comp_Y(startTime/Ts+1:endTime/Ts+1,1),'constant');
signals(:,11) = detrend(VD_Gyro_Front_Comp_Z(startTime/Ts+1:endTime/Ts+1,1),'constant');


%% Vergleichsplot vor Korrektur <-> nach Korrektur
for i = 3:11
    figure(i)
    p(i,:) = polyfit(signals(:,2),signals(:,i),1);
    plot(signals(:,1), signals(:,i));
    hold all
    plot(signals(:,1),signals(:,i)-p(i,1)*signals(:,2));

    stdev(i) = std(signals(:,i));
    stddev_corr(i) = std(signals(:,i)-p(i,1)*signals(:,2));
end
