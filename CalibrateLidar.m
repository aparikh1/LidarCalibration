function [R_SB, T_SB] = CalibrateLidar(data, guess)

%This function determines the relative pose of a 360-degree 16-beam lidar
%mounted on a mobile platform relative to the coordinate system fixed to
%the body of the mobile platform. 

%Based on: J. Underwood, A. Hill, and S. Scheding, “Calibration of range sensor
% pose on mobile platforms,” in Proceedings of the 2007 IEEE/RSJ
% International Conference on Intelligent Robots and Systems, San
% Diego, CA, USA, October 2007.
%http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=4398971

%-------------------------------------------------------------------------%
%                   Data Collection Procedure (Prior Work)
%-------------------------------------------------------------------------%

%1. Single vertical pole placed on ~flat ground
%2. Pose of body coordinate system (B) relative to ground coordinate system (G) is
%   known/measured (roll or pitch of body must be non-zero; full constraints
%   found in referenced paper)
%3. Lidar scans environemnt, and returns 3D point cloud with points in the
%   form [x_s, y_s, z_s] in a Cartesian frame (S) fixed to the lidar
%4. Point cloud split into pole points and ground points

%-------------------------------------------------------------------------%

%Knowns/Inputs:
%data = vector of TestData objects containing all points and corresponding
%rotation matrices and translation vectors
%guess = vector of initial guesses of Euler angles and translation matrix
%from S to B = [phi10,phi20,phi30,x_sb0,y_sb0,z_sb0]'

%Outputs:
%R_SB = Rotation matrix from S to B
%T_SB = Translation matrix from S to B = [x_sb, y_sb, z_sb]'


%Define inputs to fmincon
A = [];
b = [];
Aeq = [];
beq = [];
NONLCON = [];
%LB = [-1/2*pi,-1/2*pi,-1/2*pi,-3,-3,-3];
%UB = [1/2*pi,1/2*pi,1/2*pi,3,3,3];
LB=[];
UB=[];

options = optimoptions('fmincon','Display','iter-detailed')

%Minimze the function costfun with the initial guess vector guess, lower
%bound LB and upper bound UB
[optvars,fval] = fmincon(@(vars)CostFunction(data,vars), guess, A, b, Aeq, beq, LB, UB, NONLCON, options)

%Extract Euler angles from output of fmincon output
phi=optvars(1:3);
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
R_SB = R3*R2*R1;

%Extract translation matrix
T_SB = optvars(4:6);

end








