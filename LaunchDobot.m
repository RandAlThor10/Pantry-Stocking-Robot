%% Launch Dobot Function

function Dobot = LaunchDobot()

Dobot = LinearDobot(false);

qe = [deg2rad(0) deg2rad(-90) deg2rad(0) deg2rad(0) deg2rad(0)];

Dobot.model.offset = qe;

tr1 = zeros(4,4,Dobot.model.n);
tr1(:,:,1) = Dobot.model.base;
L = Dobot.model.links;
for i = 1 : Dobot.model.n
    tr1(:,:,i+1) = tr1(:,:,i) * trotz(qr(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
end

Dobot.PlotAndColourRobot();


end
