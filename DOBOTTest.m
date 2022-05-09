
close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc





 %% Ikine James Poon


a2 = 0.135;
a3 = 0.147;


x = 0.17;    % X location
y = 0.0;      % Y location
z = 0.27 - 0.138 ;    % Z location

l = sqrt(x^2 + y^2);
D = sqrt(l^2 + z^2);

t1 = rad2deg(atan(z/l));
t2 = rad2deg(acos((a2^2 + D^2 - a3^2) / (2 * a2 * D)));

alpha = t1 + t2;
beta = rad2deg(acos((a2^2 + a3^2 - D^2) / (2 * a2 * a3)));

q1 = rad2deg(atan(y/x));
q2 = (90 - alpha);
q3r = (180 - beta - alpha);
q3m = 90 - q2 + q3r;
q4 =  q3r;
q5 = 0;


%% Launch Dobot

close all

set(0,'DefaultFigureWindowStyle','docked')
clc

R = LinearDobot(false);
                                              % Initial Joint State
                                              % Returns Joint State




qr = [ 0 0 0 0 0];

qe = [deg2rad(q1) deg2rad(q2) deg2rad(q3m) deg2rad(q4) deg2rad(q5)];

R.model.offset = qe;
r_pose = R.model.getpos()

R.PlotAndColourRobot();







% Dobot Model links
%      L(1) = Link([0      0.138      0   -pi/2      0]);
%      L(2) = Link([0      0      0.135     0      0]);
%      L(3) = Link([0      0    0.147      pi    0]);
%      L(4) = Link([0      0      0.05      pi/2  0]);
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

close all



    L(1) = Link([0      0.138  0        -pi/2  0]);
    L(2) = Link([0      0      0.135    0      0]);   
    L(3) = Link([0      0      0.147    pi      0]);
    L(4) = Link([0      0      0.075    pi/2   0]);
    L(5) = Link([0      0.1     0        0      0]);

      L(1).qlim = [-135 135]*pi/180;
      L(2).qlim = [5 80]*pi/180;
      L(3).qlim = [15 170]*pi/180;
      L(4).qlim = [-90 90]*pi/180;
      L(5).qlim = [-85 85]*pi/180;

      L(1).offset = 0;
      L(2).offset = -pi/2;

    



robot = SerialLink(L,'name','myrobot');
q = [deg2rad(q1) deg2rad(q2) deg2rad(q3m) deg2rad(q4) deg2rad(q5)];%     
qr = [0 0 0 0 0];

robot.plot(q);
robot.fkine(q)
robot.teach;

%% Limit Test

 titleStr = {'Model limits given ACTUAL real robot limits','Model limits given SUGGESTED real robot limits'};

  actualRealQ2lim = deg2rad([-5,85]);   
  suggestedRealQ2lim = deg2rad([5,80]);  %> Joint 3: Actual vs Suggested real joint limits actual joint limits less than 0 (or more than 90) cause problems with IMU (officially q2limDegs = -10 to 95 )   
  actualRealQ3lim = deg2rad([-10,95]);   
  suggestedRealQ3lim = deg2rad([5,85]); 

  
    
   
   

 qlimActualAndSuggested = {R.model.qlim,R.model.qlim}; 
  qlimActualAndSuggested{1}(2,:) = actualRealQ2lim; 
  qlimActualAndSuggested{1}(3,:) = actualRealQ3lim; 
  qlimActualAndSuggested{2}(2,:) = suggestedRealQ2lim; 
  qlimActualAndSuggested{2}(3,:) = suggestedRealQ3lim;             
                       
  fig_h = figure; 
  for limitsIndex = 1:size(titleStr,2) 
    clf(fig_h); 
    data = []; 
    lowerLimit = []; 
    upperLimit = []; 
                 
    qlim = qlimActualAndSuggested{ limitsIndex }; 
                 
    for q2 = qlim(2,1):0.01:qlim(2,2) 
        for theta3 = qlim(3,1):0.01:qlim(3,2)+0.01 
            q3 = pi/2 - q2 + theta3; 
            data = [data;q2,q3]; %#ok<AGROW> 
            if theta3 <= qlim(3,1) 
                lowerLimit = [lowerLimit;q2,q3]; %#ok<AGROW> 
            elseif qlim(3,2) <= theta3 
                upperLimit = [upperLimit;q2,q3]; %#ok<AGROW> 
            end 
        end 
    end
     plot(rad2deg(data(:,1)),rad2deg(data(:,2)),'g.'); 
    hold on; 
    plot(rad2deg(lowerLimit(:,1)),rad2deg(lowerLimit(:,2)),'b'); 
    plot(rad2deg(upperLimit(:,1)),rad2deg(upperLimit(:,2)),'r'); 
    title(titleStr{limitsIndex}) 
    set(gca,'fontSize',15) 
    xlabel('q2') 
    ylabel('q3')                 
    grid on; 
    axis([-10,90,-15,200]); 
    drawnow(); 
    img = getframe(gcf); 
    imwrite(img.cdata, [titleStr{limitsIndex}, '.jpg']); 
  end 
  

 