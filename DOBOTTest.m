
close all
set(0,'DefaultFigureWindowStyle','docked')
clc

% 
R = LinearDobot(false);
                                              % Initial Joint State
R_q = R.model.getpos();                                                 % Returns Joint State
% R.model.base = R_base;

q1 = 0;
q2 = 45;
q3 = 90 - q2 + 20;
% q4 = 180 - q2 - q3;
% q5 = 0;



qe = [deg2rad(q1) deg2rad(q2) deg2rad(80) deg2rad(0) ];%deg2rad(150) deg2rad(q5)
R.model.offset = qe;
   PlotAndColourRobot(R);

 r = R.model.n


  

    
    L(1) = Link([0      0.138  0     -pi/2   0]);
      L(2) = Link([0      0      0.135     0     0]);
  
    L(3) = Link([0      0      0.147   0      0]);
      L(4) = Link([0      0      0  0    0]);
%     L(5) = Link([0     0.05      0       0    0]);



    L(1).qlim = [-135 135]*pi/180;
     L(2).qlim = [5 80]*pi/180;
     L(3).qlim = [15 170]*pi/180;
     L(4).qlim = [-90 90]*pi/180;
%     L(5).qlim = [-85 85]*pi/180;

  L(1).offset = 0;
  L(2).offset = -pi/2;
%      L(3).offset = pi;

robot = SerialLink(L,'name','myrobot');
q = [deg2rad(q1) deg2rad(q2) deg2rad(80) deg2rad(0)];%   0
% q = [0 pi/4 pi/2 pi/4 0];
    robot.plot(q);
robot.teach;

