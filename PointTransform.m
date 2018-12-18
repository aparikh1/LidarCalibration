function [points_G, gpoints_G, R_SB] = PointTransform(tdata, phi, T_SB)

%This function is used as part of the Lidar Calibration module, and
%transforms a point in Cartesian coordinates from a coordinate system fixed
%to the lidar (S) to the ground coordinate system (G)

%Inputs:
%tdata = vector of TestData objects containing all points and corresponding
%rotation matrices and translation vectors
%phi = Euler angles describing a 123 rotation to get S from B = [phi1,
%phi2, phi3] in radians
%%T_SB = Translation matrix from S to B = [x_sb, y_sb, z_sb]'

%Outputs:
%points_G = point cloud as seen in G = [x_g1, y_g1, z_g1; x_g2,...]'
%R_SB = Rotation matrix from S to B


N=length(tdata);

%Build rotation matrix from Euler angles
%Extract Euler angles
phi1=phi(1);
phi2=phi(2);
phi3=phi(3);
%Build rotation matrices for each rotation
R1 = [1,0,0; 
      0,cos(phi1),-sin(phi1); 
      0,sin(phi1),cos(phi1)];
R2 = [cos(phi2),0,sin(phi2);
      0,1,0;
      -sin(phi2),0,cos(phi2)];
R3 = [cos(phi3),-sin(phi3),0;
      sin(phi3),cos(phi3),0;
      0,0,1];
R_SB=R3*R2*R1;

%Construct matrices for point cloud in G
% points_G = [];
% gpoints_G = [];

lp=0;
lg=0;
for a=1:N
    lp=lp+length(tdata(a).points);
    lg=lg+length(tdata(a).gpoints);
end

points_G = zeros(lp,3);
gpoints_G = zeros(lg,3);

countp=0;
countg=0;

%Transform points
for i=1:N
    M=length(tdata(i).points);
    V=length(tdata(i).gpoints);
    R_BG=tdata(i).rotMat;
    T_BG=[tdata(i).tranVec]';
    for j=1:M
        p_spi = [tdata(i).points(j,:)]';
        p_gpi = R_BG*(R_SB*p_spi + T_SB) + T_BG;
        points_G(countp+1,:) = p_gpi';
        countp=countp+1;
    end
    for k=1:V
        p_sgi = [tdata(i).gpoints(k,:)]';
        p_ggi = R_BG*(R_SB*p_sgi + T_SB) + T_BG;
        gpoints_G(countg+1,:) = p_ggi';
        countg=countg+1;
    end
end

end