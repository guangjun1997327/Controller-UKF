close all

% Start und Endzeitpunkt der Messdaten angeben
StartTime = 20;
EndTime = 560;

stopIteration = 0;
frontSensor = 0;
rearSensor = 1;

sensor = input('Which Sensor? [0 Front, 1 Rear] ');

if (sensor == frontSensor)

    % Zuerst Daten in den Nullpunkt verschieben
    x = VD_Gyro_Front_Comp_X.Data(VD_Gyro_Front_Comp_X.Time>StartTime & VD_Gyro_Front_Comp_X.Time<EndTime);
    y = VD_Gyro_Front_Comp_Y.Data(VD_Gyro_Front_Comp_Y.Time>StartTime & VD_Gyro_Front_Comp_Y.Time<EndTime);
    z = VD_Gyro_Front_Comp_Z.Data(VD_Gyro_Front_Comp_Z.Time>StartTime & VD_Gyro_Front_Comp_Z.Time<EndTime);
    
    x = x - mean(x);
    y = y - mean(y);
    z = z - mean(z);

else

    % Zuerst Daten in den Nullpunkt verschieben
    x = VD_Gyro_Rear_Comp_X.Data(VD_Gyro_Rear_Comp_X.Time>StartTime & VD_Gyro_Rear_Comp_X.Time<EndTime);
    y = VD_Gyro_Rear_Comp_Y.Data(VD_Gyro_Rear_Comp_Y.Time>StartTime & VD_Gyro_Rear_Comp_Y.Time<EndTime);
    z = VD_Gyro_Rear_Comp_Z.Data(VD_Gyro_Rear_Comp_Z.Time>StartTime & VD_Gyro_Rear_Comp_Z.Time<EndTime);

    x = x - mean(x);
    y = y - mean(y);
    z = z - mean(z);
end




while stopIteration ~= 1

% Winkel in x-Richtung aus plot bestimmen
xPlot = figure(1);
plot(x,z,'x');

% Winkel in y-Richtung aus plot bestimmen
yPlot = figure(2);
plot(y,z,'x');

% 3D Plot für generelle Übersicht
magnetic3DPlot = figure(3);
plot3(x,y,z,'x');
xlabel('x');
ylabel('y');
zlabel('z');

% Einbauposition des Gyrosensors kompensieren
phi = 8*pi/180;
theta = 5.5*pi/180;

phi = input('Drehung um x [°] ')*pi/180;
theta = input('Drehung um y [°] ')*pi/180;

Cx = [1 0 0;
      0 cos(phi) sin(phi);
      0 -sin(phi) cos(phi)
     ];
 
Cy = [cos(theta) 0 -sin(theta);
      0 1 0;
      sin(theta) 0 cos(theta)
     ];
 
 vec_corr = [x y z] * Cx * Cy;

 figure(1);
 hold all
 plot(vec_corr(:,1),vec_corr(:,3));
 hold off
 
 figure(2);
 hold all
 plot(vec_corr(:,2),vec_corr(:,3));
 hold off
 
 figure(3);
 hold all
 plot3(vec_corr(:,1),vec_corr(:,2),vec_corr(:,3));
 hold off
 
stopIteration = input('Iteration finished? [0 false, 1 true] ');

end

x = vec_corr(:,1);
y = vec_corr(:,2);
z = vec_corr(:,3);
 
% Cz = [cos(psi) sin(psi) 0;
%       -sin(psi) cos(psi) 0;
%       0 0 1
%      ];


% Ellipsoid fitten
[e_center, e_radii, e_eigenvecs, e_algebraic] = ellipsoid_fit([x, y, z],0);


% compensate distorted magnetometer data
% e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
S = [x - e_center(1), y - e_center(2), z - e_center(3)]'; % translate and make array
scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
invmap = e_eigenvecs; % inverse of above
comp = invmap * scale * map;
S = comp * S; % do compensation

% output info
fprintf( 'Ellipsoid center     :\n                   %.3g %.3g %.3g\n', e_center );
fprintf( 'Ellipsoid radii      :\n                   %.3g %.3g %.3g\n', e_radii );
fprintf( 'Ellipsoid evecs      :\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n', e_eigenvecs );
fprintf( 'Ellpisoid comp evecs :\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n', comp);

% draw data
figure;
hold on;
plot3( x, y, z, '.r' ); % original magnetometer data
plot3(S(1,:), S(2,:), S(3,:), 'b.'); % compensated data
view( -70, 40 );
axis vis3d;
axis equal;

%% draw ellipsoid fit
figure;
hold on;
plot3( x, y, z, '.r' );
maxd = max(e_radii);
step = maxd / 50;
[xp, yp, zp] = meshgrid(-maxd:step:maxd + e_center(1), -maxd:step:maxd + e_center(2), -maxd:step:maxd + e_center(3));
Ellipsoid = e_algebraic(1) *xp.*xp +   e_algebraic(2) * yp.*yp + e_algebraic(3)   * zp.*zp + ...
          2*e_algebraic(4) *xp.*yp + 2*e_algebraic(5) * xp.*zp + 2*e_algebraic(6) * yp.*zp + ...
          2*e_algebraic(7) *xp     + 2*e_algebraic(8) * yp     + 2*e_algebraic(9) * zp;
p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
set(p, 'FaceColor', 'g', 'EdgeColor', 'none');
alpha(0.5);
view( -70, 40 );
axis vis3d;
axis equal;
camlight;
lighting phong;
