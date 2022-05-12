
close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc

%




%% Launch Dobot

 close all

set(0,'DefaultFigureWindowStyle','docked')
clc

Dobot = LinearDobot(false);

qr = [ 0 0 0 0 0];

qe = [deg2rad(0) deg2rad(-90) deg2rad(0) deg2rad(0) deg2rad(0)];

Dobot.model.offset = qe;
r_pose = Dobot.model.getpos();
% Dobot.model.base = transl(0,0,0);

tr1 = zeros(4,4,Dobot.model.n);
tr1(:,:,1) = Dobot.model.base;
L = Dobot.model.links;
for i = 1 : Dobot.model.n
    tr1(:,:,i+1) = tr1(:,:,i) * trotz(qr(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

Dobot.PlotAndColourRobot();

   %% Dobot Stick Links (Not the same as the Model Link, Needs to be fixed to match)





    L(1) = Link([0      0.138  0        -pi/2  0]);
    L(2) = Link([0      0      0.135    0      0]);   
    L(3) = Link([0      0      0.147    pi      0]);
    L(4) = Link([0      0      0.075    pi/2   0]);
    L(5) = Link([0      0.1     0        0      0]);

    L(1).qlim = [-135 135]*pi/180;
    L(2).qlim = [-5 85]*pi/180;
    L(3).qlim = [-5 85]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-85 85]*pi/180;

    L(1).offset = 0;
    L(2).offset = -pi/2;

    



robot = SerialLink(L,'name','myrobot');
% q = [deg2rad(45) deg2rad(q2) deg2rad(q3m) deg2rad(q4) deg2rad(q5)];     
qr = [deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0) deg2rad(0)]; % Default Upright Pose


tr = zeros(4,4,robot.n);
tr(:,:,1) = robot.base;
L = robot.links;
for i = 1 : robot.n
    tr(:,:,i+1) = tr(:,:,i) * trotz(qr(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end



robot.plot(qr);
robot.fkine(qr);
robot.teach;




%% Objects

hold on;


 Cerealbox = BasicObject('Cereal_Box.ply',[0.1,0.3,0.0],1);

 Shelf = BasicObject('Shelf.ply',[0.3,0,0],0);

%  move(Cerealbox,[0.1,0.2,0.1]);





%% Testing Animation for model Dobot


% Move to Cereal Box

DOB_T = transl(0.1,0.3,0.18);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location




% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    
    drawnow()
end


% Transport Cereal Box

DOB_T = transl(0.25,0.1,0.18);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location


% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end



%% Testing Animation for stick Dobot
% 
% 
% 
% DOB_T2 = transl(0.1,0.2,0.2);         % Generating T-matrix
% DOB_fk2 = robot.fkine(qr); 
% DOB_q2 = JPikine(DOB_T2);                 % Generating Joint States                                                % Joint Angles for Brick Pickup Location (Brick 1)  
% Trej2 = jtraj(robot.getpos,DOB_q2,50);    % Creating trajectory                                                                  % Trajectory from Start to Pick-up Location
% 
% 
% 
% 
% % Animate to Location
% for i = 1:50
%     robot.animate(Trej2(i,:));
%     
%     DOB_tr2 = robot.fkine(Trej2(i,:));
%     
%     drawnow()
% end


 %% Ikine James Poon

function [newQ] = JPikine(EEmatrix)


a2 = 0.135;                % Length from Joint 2 to Joint 3
a3 = 0.147;                % Length from Joint 3 to Joint 4

EEx = EEmatrix(1,4)              % End Effector X position
EEy = EEmatrix(2,4);              % End Effector Y position
EEz = EEmatrix(3,4);              % End Effector Z position



x = EEx - 0.075*cos(atan(EEy/EEx))              % X location for Joint 4 from Joint 2
y = EEy - 0.075*sin(atan(EEy/EEx))              % Y location for Joint 4 from Joint 2
z = EEz + 0.1 - 0.138                           % Z location for Joint 4 from Joint 2

l = sqrt(x^2 + y^2);
D = sqrt(l^2 + z^2);

t1 = rad2deg(atan(z/l));
t2 = rad2deg(acos((a2^2 + D^2 - a3^2) / (2 * a2 * D)));

alpha = t1 + t2
beta = rad2deg(acos((a2^2 + a3^2 - D^2) / (2 * a2 * a3)));

q1 = rad2deg(atan(y/x))
q2 = (90 - alpha)
q3r = (180 - beta - alpha);
q3m = 90 - q2 + q3r;
q4 =  q3r;
q5 = 0;

newQ(1,1) = deg2rad(q1);
newQ(1,2) = deg2rad(q2);
newQ(1,3) = deg2rad(q3m);
newQ(1,4) = deg2rad(q4);
newQ(1,5) = deg2rad(q5);

end
 