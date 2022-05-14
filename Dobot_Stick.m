 %% Dobot Stick Links (Not the same as the Model Link, Needs to be fixed to match)


hold on;


    L(1) = Link([0      -0.138  0        pi/2  0]);
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

robot.base = robot.base * trotx(pi)

robot.plot(qr);
robot.fkine(qr);
robot.teach;

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



