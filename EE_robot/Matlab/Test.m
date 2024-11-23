initial_angles = [0, pi/2, -pi/2, -pi/2, pi/2, 0]';
distances = [1.5,7.2,3,7,0.95,2.5,3.6,0];

[~,transform_matrix] = FindingJacobianMatrixandTransformationMatrix(initial_angles, distances);

x_initial = [transform_matrix(1,4),transform_matrix(2,4),transform_matrix(3,4)]';
pitch = -asin(transform_matrix(3,1));
yaw = atan(transform_matrix(2,1)/cos(pitch)/transform_matrix(1,1)/cos(pitch));

disp(x_initial);
disp(pitch);
disp(yaw);