%% Launch Dobot

% Add robotics toolbox to path
% Add Dobot_PLY_files to path - For dobot model
% Add Environment_files to path - For objects



close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc



% Launch Dobot
Dobot = LaunchDobot([0,0,0]);



% Set up Environemnt

hold on;
axis([-0.6 0.6,-0.6 0.6,0 0.7]);
surf([-0.6,-0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0,0;0,0],'CData',imread('floor.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[-0.6,0.6;-0.6,0.6],[0.7,0.7;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([-0.6,0.6;-0.6,0.6],[-0.6,-0.6;-0.6,-0.6],[0.7,0.7;0,0],'CData',imread('Wall.jpg'),'FaceColor','texturemap');
surf([0.6,0.6;0.6,0.6],[0.5,0.3;0.5,0.3],[0.4,0.4;0.25,0.25],'CData',imread('Warning_sign.jpg'),'FaceColor','texturemap');

% Objects

hold on;


% Moveable
Cerealbox = BasicObject('Cereal_Box.ply',[-0.1,0.3,0.0],1,[0.34,0.015,0.125]);

Muselibox = BasicObject('Museli_Box.ply',[-0.175,0.2,0],1,[]);

SauceBottle = BasicObject('Sauce_Bottle.ply',[-0.25,0.25,0],1,[]);


% Environment
Shelf1 = BasicObject('Shelf.ply',[0.34,0,0],0,[]);

Shelf2 = BasicObject('Shelf_2.ply',[0.035,-0.34,0],0,[]);

Fire_Extinguisher = BasicObject('Fireextinguisher.ply',[0.3,0.25,0],0,[]);

Barrier1 = BasicObject('Barrier_1.ply',[0.2,0.4,0],0,[]);

Barrier2 = BasicObject('Barrier_1.ply',[-0.2,0.4,0],0,[]);

Barrier3 = BasicObject('Barrier_2.ply',[-0.4,-0.2,0],0,[]);

Barrier4 = BasicObject('Barrier_2.ply',[-0.4,0.2,0],0,[]);

Barrier5 = BasicObject('Barrier_2.ply',[0.45,0.2,0],0,[]);

Barrier6 = BasicObject('Barrier_1.ply',[-0.2,-0.5,0],0,[]);

Shelf_Box1 = BasicObject('Stationary_Box.ply',[0.34,-0.1,0],0,[]);

Shelf_Box2 = BasicObject('Stationary_Box.ply',[0.34,-0.17,0],0,[]);

Box = BasicObject('Box.ply',[-0.15,0.25,0],0,[]);





%% Testing Animation for model Dobot


% Move to Cereal Box
% %TEST


 DOB_T = transl(0.2,0,0.125)  ;
            DOB_q = JPikine(DOB_T); 
            Trej = jtraj(Dobot.model.getpos,DOB_q,200);
            % Animate to Location
            for i = 1:200
                Dobot.model.animate(Trej(i,:));
                
                DOB_tr = Dobot.model.fkine(Trej(i,:));
                
                drawnow()
            end


DOB_T = transl(Cerealbox.location(1,1),Cerealbox.location(1,2),Cerealbox.location(1,3)+0.125);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states   

T = Dobot.model.fkine(Dobot.model.getpos);
[Traj, steps] =  PlanTraj(T(1:3,4)', SauceBottle.location, Dobot.model, Dobot.model.getpos);


Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                               

% Animate to Location
for i = 1:steps
    Dobot.model.animate(Traj(i,:));
    
    DOB_tr = Dobot.model.fkine(Traj(i,:));
    
    drawnow()
end


% Transport Cereal Box

% Intermediate Point
DOB_T = transl(Cerealbox.location(1,1),0.25,0.18);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                             
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                             

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end



% Intermediate Point
DOB_T = transl(0.15,0.175,0.125);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                        
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                           

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end





% Final Point
DOB_T = transl(0.34,0.015,0.125);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                            
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                              


% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    attach(Cerealbox,DOB_tr);
    drawnow()
end

% Move Back



DOB_T = transl(0.2,0.1,0.05);           % Generating T-matrix
DOB_q = JPikine(DOB_T);                   % Generating Joint states                                        
Trej = jtraj(Dobot.model.getpos,DOB_q,200);    % Creating trajectory                                                            

% Animate to Location
for i = 1:200
    Dobot.model.animate(Trej(i,:));
    
    DOB_tr = Dobot.model.fkine(Trej(i,:));
    
    drawnow()
end


 