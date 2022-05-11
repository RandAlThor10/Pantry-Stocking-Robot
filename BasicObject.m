%generate all environment in classes 
%test
classdef BasicObject
   properties
      location
      currentvertices
      originvertices
      type %Environment or pantry item
   end
   methods
      function obj = BasicObject(val1, val2)
          obj.location = val1;
          obj.originvertices = val2;
          obj.currentvertices = [obj.originvertices, ones(size(obj.originvertices,1),1)]*transl(obj.location)
       end
   
      function move(obj,[newx,newy,newz],rotz)
         %move the location and meshs to a new position
         obj.location = [newx,newy,newz]
         obj.currentvertices = [obj.originvertices, ones(size(obj.originvertices,1),1)]*transl(newx,newy,newz)
      end
   end
end
