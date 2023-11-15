factor=1.167;
YawRate_Calc=YawRate_Calculated_simout.Data;
YawRate_Orig=YawRate_simout.Data*factor;
figure
hold on
plot(YawRate_Calc,'g');
plot(YawRate_Orig,'r');
ylim([-2 2]);
