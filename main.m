clc
clear

%Load robot

Dobot = LaunchDobot();

%Load the Environment

hold on;
axis([-0.6 0.6,-0.6 0.6,0 0.6]);
surf([-0.6,-0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0,0;0,0],'CData',imread('floor.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0.6,0.6;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([-0.6,0.6;-0.6,0.6],[-0.6,-0.6;-0.6,-0.6],[0.6,0.6;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[0.5,0.3;0.5,0.3],[0.4,0.4;0.25,0.25],'CData',imread('Warning_sign.jpg'),'FaceColor','texturemap');


% Objects

hold on;

Cerealbox = BasicObject('Cereal_Box.ply',[-0.1,0.3,0.0],1);

Shelf1 = BasicObject('Shelf.ply',[0.3,-0.1,0],0);

Shelf2 = BasicObject('Shelf_2.ply',[0.035,-0.35,0],0);

Fire_Extinguisher = BasicObject('Fireextinguisher.ply',[0.3,0.25,0],0);

SauceBottle = BasicObject('Sauce_Bottle.ply',[-0.25,0.2,0],0);

Barrier1 = BasicObject('Barrier_1.ply',[0.2,0.4,0],0);

Barrier2 = BasicObject('Barrier_1.ply',[-0.2,0.4,0],0);

Barrier3 = BasicObject('Barrier_2.ply',[-0.4,-0.2,0],0);

Barrier4 = BasicObject('Barrier_2.ply',[-0.4,0.2,0],0);

Barrier5 = BasicObject('Barrier_2.ply',[0.4,0.2,0],0);

Barrier6 = BasicObject('Barrier_1.ply',[-0.2,-0.5,0],0);

Box = BasicObject('Box.ply',[-0.15,0.25,0],0);

Objarray = [Cerealbox, Shelf1, Shelf2, Fire_Extinguisher, SauceBottle, Barrier1 , Barrier2, Barrier3, Barrier4, Barrier5, Barrier6, Box];

%run the robot
moving = false;

while any([Objarray.type]) %continue program until all objects are type 0
    T = Dobot.model.fkine(Dobot.model.getpos);
%Look at GUI

%DobotGUI()  %Ashish calls gui to check for input?

%Stock pantry
    if moving == true
        Dobot.model.animate(Traj(1,:));
        Traj(1,:) = [];
    elseif moving == false 
        %set xyz target, immediately above object to grasp
        if any([Objarray.type])
            %no items left to move, stop moving
        else
            %move to the next location
            for i = 1:size(Objarray,2) %find closest object to be moved
                if Objarray(i).type == 0 %obj that doesn't need to be moved
                    distance(i) = 9001;
                else
                    distance(i) = sqrt(sum((T(1:3,4)'-Objarray(i).location.^2)));
                end
            end
            [~,targetindex] = min(distance);
            [Traj, steps, target] =  PlanTraj(T(1:3,4)',target, Objarray(targetindex).location,Dobot.model, Dobot.model.getpos);
            
        end
    elseif moving == "manual"
        
    %Move robot manually

    end
  pause(0.001)  
end


%Show on GUI stopping robot then shutdown gui
