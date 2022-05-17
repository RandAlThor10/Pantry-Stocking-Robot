%generate all environment in classes 
%test
classdef BasicObject
    properties
      location
      endlocation
      grasplocation
      waypoint
      rotation
      vertices
      transformedvertices
      ply
      type %Static = 0 or pantry item to be moved = 1
    end
    methods
        function obj = BasicObject(val1,val2,val3,val4,val5)
          obj.ply = PlaceObject(val1);
          obj.location = val2;
          obj.type = val3;
          obj.endlocation = val4;
          obj.vertices = get(obj.ply,'Vertices');

          if obj.type == 1
          obj.grasplocation = obj.location;
          obj.grasplocation(1,3) = obj.endlocation(1,3);
          obj.waypoint = val5;
          end
          
          temp = [obj.vertices, ones(size(obj.vertices,1),1)] *transl(obj.location(1,1),obj.location(1,2),obj.location(1,3))';
          obj.transformedvertices = temp(:,1:3);
          set(obj.ply,'Vertices',obj.transformedvertices(:,1:3));
          
         
        end
   
        function move(obj,newlocation) % i.e. newlocation = [0.1,0.2,0.3]
         %move the location and meshs to a new position
         obj.location = newlocation;
         obj.grasplocation = newlocation;
         obj.grasplocation(1,3) = obj.endlocation(1,3);
         obj.transformedvertices = [obj.vertices, ones(size(obj.vertices,1),1)] *transl(obj.location(1,1),obj.location(1,2),obj.location(1,3))';
         set(obj.ply,'Vertices',obj.transformedvertices(:,1:3));
        end

     
        function attach(obj,EEmatrix) % Used for transportation by robot

         obj.location = EEmatrix;
         obj.transformedvertices = [obj.vertices, ones(size(obj.vertices,1),1)] *obj.location';
         set(obj.ply,'Vertices',obj.transformedvertices(:,1:3));



        end    
    end
end
