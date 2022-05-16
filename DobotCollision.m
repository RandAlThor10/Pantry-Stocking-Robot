%function for stopping robot when there is collision
function [collision] = DobotCollision(traj,model,Objarray) 


collision = false;
for i = 1:6 %check every link mesh for collision
    for a = 1:size(Objarray,2)
        %check link i for collision with mesh a
        Objarray(a).transformedvertices
        if collision == true
            collision = true; 
        end
    end
end
end
