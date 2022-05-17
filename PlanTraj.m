%function for planning the dobot movement in XYZ avoiding obstacles
function [traj, steps] = PlanTraj(start,finish,model,startpose) 
%start(x,y,z), finish(x,y,z), kinematic model, startpose (q)

speed = 0.001; %approx speed in m/s
distance = sqrt(sum((start-finish).^2));
steps = ceil(distance/speed);
deltaT = distance/speed;
epsilon = 0.01;      % Threshold value for manipulability/Damped Least Squares
W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

% 1.2) Allocate array data
m = zeros(steps,1);             % Array for Measure of Manipulability
qMatrix = zeros(steps,5);       % Array for joint anglesR
qdot = zeros(steps,5);          % Array for joint velocities
theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
x = zeros(3,steps);             % Array for x-y-z trajectory
positionError = zeros(3,steps); % Trajectory error
angleError = zeros(3,steps);    

% 1.3) Set up trajectory, initial pose
s = lspb(0,1,steps);            % Trapezoidal trajectory scalar
for i=1:steps
    x(1,i) = (1-s(i))*start(1) + s(i)*finish(1); % Points in x
    x(2,i) = (1-s(i))*start(2) + s(i)*finish(2); % Points in y
    x(3,i) = (1-s(i))*start(3) + s(i)*finish(3); % Points in z
    theta(1,i) = 0;             % Roll angle 
    theta(2,i) = 0;             % Pitch angle
    theta(3,i) = 0;             % Yaw angle
end


qMatrix(1,:) = JPikine(model.fkine(startpose));   %First joint state

% 1.4) Track the trajectory with RMRC
for i = 1:steps-1
    T = model.fkine(qMatrix(i,:));                                          % Get forward transformation at current joint state
    deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
    Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
    Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
    Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
    S = Rdot*Ra';                                                           % Skew symmetric!
    linear_velocity = (1/deltaT)*deltaX;
    angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
    deltaTheta = tr2rpy(Rd*Ra');                                            % Convert rotation matrix to RPY angles
    xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
    J = model.jacob0(qMatrix(i,:));                 % Get Jacobian at current joint state
    J2 = J(1:3,1:3);
    m(i) = sqrt(det(J2*J2'));
    if m(i) < epsilon  % If manipulability is less than given threshold
        lambda = (1 - m(i)/epsilon)*5E-2;
    else
        lambda = 0;
    end
    invJ = inv(J'*J + lambda *eye(5))*J';                                   % DLS Inverse
    qdot(i,:) = (invJ*xdot)';                                                % Solve the RMRC equation (you may need to transpose the vector)
    for j = 1:5                                                             % Loop through joints 1 to 5
        if qMatrix(i,j) + deltaT*qdot(i,j) < model.qlim(j,1)                     % If next joint angle is lower than joint limit...
            qdot(i,j) = 0; % Stop the motor
        elseif qMatrix(i,j) + deltaT*qdot(i,j) > model.qlim(j,2)                 % If next joint angle is greater than joint limit ...
            qdot(i,j) = 0; % Stop the motor
        end
    end
%      T = model.fkine(qMatrix(i,:) + deltaT*qdot(i,:));
%     qMatrix(i+1,:) = JPikine(T);
     
    
    qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                         	% Update next joint state based on joint velocities
    
    positionError(:,i) = x(:,i+1) - T(1:3,4);                               % For plotting
    angleError(:,i) = deltaTheta;                                           % For plotting
end

traj  = qMatrix;

%perform collision detection

% = DobotCollision();
%if collision with objects
    %traj1 = PlanTraj to pose that avoids it
    %traj2 = PlanTraj from waypoint pos to finish
    %traj = traj 1 & 2 joined


end
