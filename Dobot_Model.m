%% Launch Dobot

% Add robotics toolbox to path
% Add Dobot_PLY_files to path - For dobot model
% Add Environment_files to path - For objects



close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc

% Set up Environemnt


                                                                

hold on;
axis([-0.6 0.6,-0.6 0.6,0 0.6]);
surf([-0.6,-0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0,0;0,0],'CData',imread('floor.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0.6,0.6;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([-0.6,0.6;-0.6,0.6],[-0.6,-0.6;-0.6,-0.6],[0.6,0.6;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');



% Launch Dobot
Dobot = LaunchDobot();


% Objects

hold on;


Cerealbox = BasicObject('Cereal_Box.ply',[0.0001,0.3,0.0],1);

Shelf1 = BasicObject('Shelf.ply',[0.3,0,0],0);

Shelf2 = BasicObject('Shelf_2.ply',[0,-0.3,0],0);

Fire_Extinguisher = BasicObject('Fireextinguisher.ply',[0.3,0.25,0],0); 
% Box = BasicObject('Box.ply',[-0.15,0.2,0],0);






%% Testing Animation for model Dobot


% Move to Cereal Box

DOB_T = transl(0.0001,0.3,0.12);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    
    drawnow()
end


% Transport Cereal Box

% Intermediate Point
DOB_T = transl(0.175,0.1,0.12);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end

% Final Point
DOB_T = transl(0.3,0.1,0.12);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location


% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end

% Move Back



DOB_T = transl(0.175,0.1,0.12);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                              % Joint Angles for Brick Pickup Location (Brick 1)  
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                                % Trajectory from Start to Pick-up Location

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    
    drawnow()
end


 