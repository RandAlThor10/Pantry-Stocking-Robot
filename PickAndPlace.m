function PickAndPlace(Dobot,Objarray)

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
        NextObj = false;
    %Move robot manually

    end
  pause(0.001)  
end
