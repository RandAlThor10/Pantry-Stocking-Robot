function [cam,warning] = DobotCam(setup, model, cam)

%% Setup
setup = 1;
warning = false;
if setup == 1
% Add the camera
    cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
    'resolution', [1024 1024], 'centre', [512 512],'name', 'DobotCamera');

end
%% Update
cam.T = model.fkine(model.getpos);
%cam.plot_camera('scale',0.02);

%% 1.3 Initialise Simulation (Display in Image view)

%3d Warning points
Warning = [0.6,0.3,0.25;0.6,0.5,0.25;0.6,0.5,0.4;0.6,0.3,0.4]';
Centrepoint = [512;512];
%camera view and plotting
cam.clf()
uv = cam.plot(Warning,'Tcam',cam.T,'*');
cam.hold(true);
cam.plot(Centrepoint,'Tcam',cam.T,'o');
in = inpolygon(Centrepoint(1), Centrepoint(2), uv(1,:), uv(2,:));
dist = sqrt(sum((cam.centre()'-[0.6,0.4,0.325]).^2)); %distance of cam from sign
if distance < 0.1 && in == 1
    warning = true;
end
