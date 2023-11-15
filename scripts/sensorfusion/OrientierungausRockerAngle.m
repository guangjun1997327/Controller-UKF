motion_ratio_front=0.0008602;
motion_ratio_rear=0.0009917;
l_F=0.7941;
l_R=0.7459;
Spurweite=1.2;

a11=1;
a12=-l_F;
a13=-0.5*Spurweite;
a21=1;
a22=-l_F;
a23=0.5*Spurweite;
a31=1;
a32=l_R;
a33=-0.5*Spurweite;

A=[a11, a12, a13; a21, a22, a23; a31, a32, a33];
for i=1:1:length(RockerAngle_FL.Data)
b1=RockerAngle_FL.Data*motion_ratio_front;
b2=RockerAngle_FR.Data*motion_ratio_front;
b3=RockerAngle_RL.Data*motion_ratio_rear;
b=[b1(:), b2(:), b3(:)];

b=b(i,:);
x=A\b'; 
y=x';
C(i,:)=[y];
end

% syms x y z  
% a=solve(b1(5) == a11*x + a12*y+a13*z, b2(5)== x +a22*y+a23*z, b3 == x +a32*y+a33*z); 
f=RockerAngle_FL.Time;
PosZ=timeseries( C(:,1),RockerAngle_FL.Time);
PitchAngle=timeseries( C(:,2),RockerAngle_FL.Time);
RollAngle=timeseries( C(:,3),RockerAngle_FL.Time);

Stopzeit=125;
figure
% hold on;
subplot(3,1,1)

plot(PosZ_kalman, 'color','red');
hold on;
plot(PosZ, 'color','blue');

legend('PosZ kalman', 'PosZ');
axis([0,Stopzeit,-0.005,0.005]);
title('Position Z');
xlabel('Time in sec');
ylabel('Position in Meter');
grid on;

subplot(3,1,2)

plot(RollAngle_kalman*57.3, 'color','cyan');
hold on;
plot(RollAngle*57.3, 'color','blue');
legend('kalman', 'GS');
axis([0,Stopzeit,-3,3]);
title('RollAngle');
xlabel('Time in sec');
ylabel('Angle in Degree');
grid on;

subplot(3,1,3)

plot(PitchAngle_kalman*57.3, 'color','green');
hold on;
plot(PitchAngle*57.3, 'color','blue');

legend('kalman','GS')
axis([0,Stopzeit,-3,3]);
title('PitchAngle');
xlabel('Time in sec');
ylabel('Angle in Degree');
grid on;
