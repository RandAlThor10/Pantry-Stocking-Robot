close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc

%Load robot

Dobot = LaunchDobot([0,0,0]);

%Load the Environment

hold on;
axis([-0.6 0.6,-0.6 0.6,0 0.7]);
surf([-0.6,-0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0,0;0,0],'CData',imread('floor.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0.7,0.7;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([-0.6,0.6;-0.6,0.6],[-0.6,-0.6;-0.6,-0.6],[0.7,0.7;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[0.5,0.3;0.5,0.3],[0.4,0.4;0.25,0.25],'CData',imread('Warning_sign.jpg'),'FaceColor','texturemap');

% Objects

hold on;


% Moveable
Cerealbox = BasicObject('Cereal_Box.ply',[-0.1,0.3,0.0],1,[0.3,0.135,0.125],[0.2,0.125,0.13]);

Muselibox = BasicObject('Museli_Box.ply',[-0.175,0.2,0],1,[0.1,-0.33,0.11],[0.2,0,0.12]);

SauceBottle = BasicObject('Sauce_Can.ply',[-0.1,0.2,0],1,[0.3,-0.1,0.1],[0.2,0.15,0.13]);


% Environment
Shelf1 = BasicObject('Shelf.ply',[0.3,0,0],0,[],[]);

Shelf2 = BasicObject('Shelf_2.ply',[0.035,-0.32,0],0,[],[]);

Fire_Extinguisher = BasicObject('Fireextinguisher.ply',[0.3,0.25,0],0,[],[]);

Barrier1 = BasicObject('Barrier_1.ply',[0.2,0.4,0],0,[],[]);

Barrier2 = BasicObject('Barrier_1.ply',[-0.2,0.4,0],0,[],[]);

Barrier3 = BasicObject('Barrier_2.ply',[-0.4,-0.2,0],0,[],[]);

Barrier4 = BasicObject('Barrier_2.ply',[-0.4,0.2,0],0,[],[]);

Barrier5 = BasicObject('Barrier_2.ply',[0.45,0.2,0],0,[],[]);

Barrier6 = BasicObject('Barrier_1.ply',[-0.2,-0.5,0],0,[],[]);

Shelf_Box1 = BasicObject('Stationary_Box.ply',[0.34,0.17,0],0,[],[]);

Shelf_Box2 = BasicObject('Stationary_Box.ply',[0.34,0.1,0],0,[],[]);

Box = BasicObject('Box.ply',[-0.15,0.25,0],0,[],[]);



Objarray = [Cerealbox, Muselibox, SauceBottle, Shelf1, Shelf2, Fire_Extinguisher, Barrier1, Barrier2, Barrier3, Barrier4, Barrier5, Barrier6, Shelf_Box1, Shelf_Box2, Box];

%run the robot
moving = false;
transporting = true;
steps = 200;

Lift = false;
Waypoint = false;
Endlocation = false;

while any([Objarray.type]) %continue program until all objects are type 0
    T = Dobot.model.fkine(Dobot.model.getpos);
%Look at GUI

%DobotGUI()  %Ashish calls gui to check for input?

%Stock pantry
    if moving == true

      if transporting == false
         % Animate to the object
         for x = 1:steps
            Dobot.model.animate(Traj(x,:));
            drawnow()
         end
         % Once complete create new traj for moving the obj
         Lift = true;
         transporting = 1;

      elseif transporting == true

        if Lift == true
          % Lifting Point
          DOB_T = transl(Objarray(targetindex).location(1,1),Objarray(targetindex).location(1,2),0.18);           % Generating T-matrix
          DOB_q = JPikine(DOB_T); 
          Traj = jtraj(Dobot.model.getpos,DOB_q,steps);
          Lift = false;
          Waypoint = true;

        elseif Waypoint == true
          %  Intermediate Waypoint
          DOB_T = transl(Objarray(targetindex).waypoint(1,1),Objarray(targetindex).waypoint(1,2),Objarray(targetindex).waypoint(1,3));           % Generating T-matrix
          DOB_q = JPikine(DOB_T); 
          Traj = jtraj(Dobot.model.getpos,DOB_q,steps);
          Waypoint = false;
          Endlocation = true;

        elseif Endlocation == true
          % Final Location
          DOB_T = transl(Objarray(targetindex).endlocation(1,1),Objarray(targetindex).endlocation(1,2),Objarray(targetindex).endlocation(1,3));           % Generating T-matrix
          DOB_q = JPikine(DOB_T); 
          Traj = jtraj(Dobot.model.getpos,DOB_q,steps);
          Endlocation = false;

        end

       
       
        
       % Animate
       for x = 1:steps
            Dobot.model.animate(Traj(x,:));
            DOB_tr = Dobot.model.fkine(Traj(x,:));
            attach(Objarray(targetindex),DOB_tr);
            drawnow()
       end
        
      
       
       transporting = false;
       moving = false;
       Objarray(targetindex).type = 0;                                      % Become Static
       

     end
       % move until finished
    elseif moving == false 
        %set xyz target, immediately above object to grasp
      
            %move to the next object
            for i = 1:size(Objarray,2) %find closest object to be moved
                if Objarray(i).type == 0 %obj that doesn't need to be moved
                    distance(i) = 9001;
                else 
                    distance(i) = sqrt(sum((T(1:3,4)'-Objarray(i).location).^2));
                end
            end
            [~,targetindex] = min(distance);
            % Found closest Obj
            transporting = 0;
            % Trajectory to the  Obj location
            
            currentT = Dobot.model.fkine(Dobot.model.getpos); 
            DOB_T = transl(Objarray(targetindex).location(1,1),Objarray(targetindex).location(1,2),Objarray(targetindex).grasplocation(1,3));           % Generating T-matrix
            DOB_q = JPikine(DOB_T);                                         % Generating Joint states  
            
            %[Traj, steps] =  PlanTraj(A(1:3,4)', Objarray(targetindex).location, Dobot.model, Dobot.model.getpos);
            
            Traj = jtraj(Dobot.model.getpos,DOB_q,steps);                      % Creating trajectory

            moving = true;
     
    elseif moving == "manual"
        
    %Move robot manually

    end
  pause(0.001)  
end


%Show on GUI stopping robot then shutdown gui
