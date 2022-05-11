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
      function move(obj,[newx,newy,newz],rotz)
         %move the location and meshs to a new position
         obj.location = [newx,newy,newz]
         obj.currentvertices = [originvertices, ones(size(originvertices,1),1)]*transl(newx,newy,newz)
      end
   end
end
