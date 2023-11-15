%% fit ellipsoid
%if front
if(1) % 0 = rear , 1= front
%  CompX = VD_Gyro_Front_Comp_X.Data;
%  CompY = VD_Gyro_Front_Comp_Y.Data;
%  CompZ = VD_Gyro_Front_Comp_Z.Data;
 
CompX = VD_Gyro_Front_Comp_X.Data(VD_Gyro_Front_Comp_Y.Time>160 & VD_Gyro_Front_Comp_Y.Time<255);
CompY = VD_Gyro_Front_Comp_Y.Data(VD_Gyro_Front_Comp_Y.Time>160 & VD_Gyro_Front_Comp_Y.Time<255);
CompZ = VD_Gyro_Front_Comp_Z.Data(VD_Gyro_Front_Comp_Z.Time>160 & VD_Gyro_Front_Comp_Y.Time<255);

else
%if rear
CompX = VD_Gyro_Rear_Comp_X.Data(VD_Gyro_Rear_Comp_Y.Time>200 & VD_Gyro_Rear_Comp_Y.Time<260);
CompY = VD_Gyro_Rear_Comp_Y.Data(VD_Gyro_Rear_Comp_Y.Time>200 & VD_Gyro_Rear_Comp_Y.Time<260);
CompZ = VD_Gyro_Rear_Comp_Z.Data(VD_Gyro_Rear_Comp_Z.Time>200 & VD_Gyro_Rear_Comp_Y.Time<260);

end
x = [CompX, CompY, CompZ];
[e_center, e_radii, e_eigenvecs, e_algebraic] = ellipsoid_fit(x);


%% convert measurements
CompX = CompX - e_center(1);
CompY = CompY - e_center(2);
CompZ = CompZ - e_center(3);

%S = [x - e_center(1), y - e_center(2), z - e_center(3)]'; % translate and make array
Comp=[CompX'; CompY'; CompZ'];
scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
invmap = e_eigenvecs; % inverse of above
comp = invmap * scale*map;
CompCorr = comp * Comp; % do compensation


plot3(CompCorr(1,:), CompCorr(2,:), CompCorr(3,:))
hold on
%plot3(Comp(1,:), Comp(2,:), Comp(3,:))
%rectangle('Position', [-0.17 -0.17 0.34 0.34], 'Curvature', [1 1])
axis equal;


%% calculate heading
figure;

% plot(CompCorr);