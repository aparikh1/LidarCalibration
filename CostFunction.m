function [c] = CostFunction(testdata, vars)

%This function evaluates the cost function laid out in
%(http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4398971) to be
%minimized.

%Inputs:
%testdata = vector of TestData objects containing all points and
%corresponding rotation matrices and translation vectors
%vars = vector of Euler angles and translation matrix from S to B = [phi1,
%phi2, phi3, x_sb, y_sb, z_sb]'

%Outputs:
%c = cost


%Extract Euler angles and translation matrix
phi = vars(1:3);
T_SB = vars(4:6);

%Transform point cloud from S to G
[P_G, gP_G, R_SB] = PointTransform(testdata, phi, T_SB);

%Initialize parameters
sig_px = 0;
sig_py = 0;
sig_gz = 0;

[Mp,Np] = size(P_G);
%Extract x, y, and z Cartesian data, and means
x_G = P_G(:,1);
y_G = P_G(:,2);
gz_G= gP_G(:,3);

%Determine variances
sig_px = var(x_G, 1);
sig_py = var(y_G, 1);
sig_gz = var(gz_G, 1);

%Determine Cost
c = sig_px + sig_py + sig_gz;

end




