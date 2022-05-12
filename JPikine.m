 %% Ikine James Poon

function [newQ] = JPikine(EEmatrix)


a2 = 0.135;                % Length from Joint 2 to Joint 3
a3 = 0.147;                % Length from Joint 3 to Joint 4

EEx = EEmatrix(1,4);              % End Effector X position
EEy = EEmatrix(2,4);              % End Effector Y position
EEz = EEmatrix(3,4);              % End Effector Z position


% if EEx < 0
% x = EEx + 0.075*cos(atan(EEy/EEx));              % X location for Joint 4 from Joint 2
% else
x = EEx - 0.075*cos(atan(EEy/EEx));  
% end

% if EEy > 0
% y = EEy + 0.075*sin(atan(EEy/EEx));              % Y location for Joint 4 from Joint 2
% else
y = EEy - 0.075*sin(atan(EEy/EEx));
% end

z = EEz + 0.1 - 0.138;                           % Z location for Joint 4 from Joint 2

l = sqrt(x^2 + y^2);
D = sqrt(l^2 + z^2);

t1 = rad2deg(atan(z/l));
t2 = rad2deg(acos((a2^2 + D^2 - a3^2) / (2 * a2 * D)));

alpha = t1 + t2;
beta = rad2deg(acos((a2^2 + a3^2 - D^2) / (2 * a2 * a3)));

q1 = rad2deg(atan(y/x));
q2 = (90 - alpha);
q3r = (180 - beta - alpha);
q3m = 90 - q2 + q3r;
q4 =  q3r;
q5 = q1;

newQ(1,1) = deg2rad(q1);
newQ(1,2) = deg2rad(q2);
newQ(1,3) = deg2rad(q3m);
newQ(1,4) = deg2rad(q4);
newQ(1,5) = deg2rad(q5);

end
 