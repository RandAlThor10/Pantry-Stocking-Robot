clc
clear
%Zac was here
%Load robot



%Load the Environment

%Item(i) = BasicObject([x,y,z],vertices,type)



%run the robot
moving = false;

while true

%Look at GUI

%DobotGUI()  %Ashish calls gui to check for input?

%Stock pantry
    if moving == true
        Dobot.model.animate(Traj(1,:));
        Traj(1,:) = [];
    elseif moving == false 
        %set xyz target, immediately above object to grasp
        if Items == []
            %no items left to move, return to default pose then stop moving
        else
            %move to the next location
            [Traj, steps, target] =  PlanTraj(,,Dobot.model, Dobot.model.getpos) 
        end
    elseif moving == "manual"
        
    %Move robot manually

    end
end


%Show on GUI stopping robot then shutdown gui
