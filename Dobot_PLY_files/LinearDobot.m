classdef LinearDobot < handle
    properties
        %> Robot model
        model;
        
        %>
        workspace = [-1 1 -1 1 -0.3 1];   
        
        %> Flag to indicate if gripper is used
        useGripper = false;        
    end
    
    methods%% Class for Dobot robot simulation
function self = LinearDobot(useGripper)
    self.useGripper = useGripper;
    
%> Define the boundaries of the workspace

        
% robot = 
self.GetDobot();
% robot = 
self.PlotAndColourRobot();%robot,workspace);
end

%% GetDobot
% Given a name (optional), create and return a Dobot robot model
function GetDobot(self)
%     if nargin < 1
        % Create a unique name (ms timestamp after 1ms pause)
        pause(0.001);
        name = ['LinearDobot_',datestr(now,'yyyymmddTHHMMSSFFF')];
%     end


    L(1) = Link([0      -0.138  0       pi/2  0]);
    L(2) = Link([0      0      0.135    0      0]);   
    L(3) = Link([0      0      0.147    pi      0]);
    L(4) = Link([0      0      0.075    pi/2   0]);
    L(5) = Link([0     0.1     0        0      0]);



    % Incorporate joint limits
    
   
    L(1).qlim = [-135 135]*pi/180;
    L(2).qlim = [-5 85]*pi/180;
    L(3).qlim = [-5 85]*pi/180;
    L(4).qlim = [-90 90]*pi/180;
    L(5).qlim = [-85 85]*pi/180;

    L(2).offset = 0;
    L(3).offset = -pi/2;


 


    self.model = SerialLink(L,'name',name);
  
    % Rotate robot to the correct orientation

   self.model.base = self.model.base  * trotx(pi);

end
%% PlotAndColourRobot
% Given a robot index, add the glyphs (vertices and faces) and
% colour them in if data is available 
function PlotAndColourRobot(self)%robot,workspace)
    for linkIndex = 0:self.model.n
%         disp(['Index',num2str(linkIndex)]);
        if self.useGripper && linkIndex == self.model.n
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Dobotjoint',num2str(linkIndex),'Gripper.ply'],'tri'); %#ok<AGROW>
        else
            [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['Dobotjoint',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
%             disp(['Joint',num2str(linkIndex)]);
        end
        self.model.faces{linkIndex+1} = faceData;
        self.model.points{linkIndex+1} = vertexData;
    end

    % Display robot
    self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
    if isempty(findobj(get(gca,'Children'),'Type','Light'))
        camlight
    end  
    self.model.delay = 0;

    % Try to correctly colour the arm (if colours are in ply file data)
    for linkIndex = 0:self.model.n
        handles = findobj('Tag', self.model.name);
        h = get(handles,'UserData');
        try 
            h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                          , plyData{linkIndex+1}.vertex.green ...
                                                          , plyData{linkIndex+1}.vertex.blue]/255;
            h.link(linkIndex+1).Children.FaceColor = 'interp';
        catch ME_1
            disp(ME_1);
            continue;
        end
    end
end        
    end
end





























