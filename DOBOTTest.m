
close all
set(0,'DefaultFigureWindowStyle','docked')
clc

q1 = 0;
q2 = 5;
q3 = 90 - q2 + 20;
q4 = 180 - q2 - q3;
q5 = 0;

%% Launch Dobot

close all
set(0,'DefaultFigureWindowStyle','docked')
clc

R = LinearDobot(false);
                                              % Initial Joint State
                                              % Returns Joint State






% qe = [deg2rad(q1) deg2rad(q2) deg2rad(90) deg2rad(45) deg2rad(q5) ];%deg2rad(150) deg2rad(q5)
R.model.offset = [ 0 -pi/2 0 0 0];
r_pose = R.model.getpos()
R.PlotAndColourRobot();







% Dobot Model links
%      L(1) = Link([0      0.138      0   -pi/2      0]);
%      L(2) = Link([0      0      0.135     0      0]);
%      L(3) = Link([0      0    0.157      0    0]);
%      L(4) = Link([0      0      0.05      -pi/2  0]);
%      L(5) = Link([0      0.1      0    0     0]);

%% Testing Animation for model

DOB_T1 = transl(0.2,0,0.1);  % Transform for Brick Location (Brick 1) 
DOB_q1 = R.model.ikcon(DOB_T1,R.model.getpos());                                                                 % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(R.model.getpos,DOB_q1,200);                                                                     % Trajectory from Start to Pick-up Location




% Animate to Bricks' Location
for i = 1:200
    R.model.animate(Trej(i,:));
    
    DOB_tr = R.model.fkine(Trej(i,:));
    
    drawnow()
end

   %% Dobot Stick Links (Not the same as the Model Link, Needs to be fixed to match)



    L(1) = Link([0      0.138  0        -pi/2  0]);
    L(2) = Link([0      0      0.135    0      0]);   
    L(3) = Link([0      0      0.147    0      0]);
    L(4) = Link([0      0      0.075     -pi/2   0]);
    L(5) = Link([0     0.1     0        0      0]);

      L(1).qlim = [-135 135]*pi/180;
      L(2).qlim = [5 80]*pi/180;
      L(3).qlim = [15 170]*pi/180;
      L(4).qlim = [-90 90]*pi/180;
      L(5).qlim = [-85 85]*pi/180;

      L(1).offset = 0;
      L(2).offset = -pi/2;



robot = SerialLink(L,'name','myrobot');
q = [deg2rad(q1) deg2rad(q2) deg2rad(90) deg2rad(45) deg2rad(q5)];%     
qr = [0 0 0 0 0];

robot.plot(qr);
robot.teach;

