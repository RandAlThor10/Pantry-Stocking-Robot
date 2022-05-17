%% Launch Dobot Function

function Dobot = LaunchDobot(location) % Location = [x,y,z] 

    Dobot = LinearDobot(false);

    qe = [deg2rad(0) deg2rad(-90) deg2rad(0) deg2rad(0) deg2rad(0)];

    Dobot.model.offset = qe;

    Dobot.model.base = transl(location(1,1),location(1,2),location(1,3)) * trotx(pi);

   

%     tr1 = zeros(4,4,Dobot.model.n);
%     tr1(:,:,1) = Dobot.model.base;
%     L = Dobot.model.links;
%     for i = 1 : Dobot.model.n
%     tr1(:,:,i+1) = tr1(:,:,i) * trotz(qr(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
%     end

   PlotAndColourRobot(Dobot);


end
