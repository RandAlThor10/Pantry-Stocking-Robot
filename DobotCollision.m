%function for checking if there is collision with the robot
function [collision] = DobotCollision(model,Objarray) 



links = model.links;
q=model.getpos;
transforms = zeros(4, 4, length(links) + 1);
transforms(:,:,1) = model.base;

for i = 1:length(links)
    L = links(1,i);
    
    current_transform = transforms(:,:, i);
    
    current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
    transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
    transforms(:,:,i + 1) = current_transform;
end


p1=zeros(size(transforms,3)-2,3);
p2=zeros(size(transforms,3)-2,3);
C=zeros(size(transforms,3)-2,3);
d=zeros(size(transforms,3)-2,3);
R=zeros(size(transforms,3)-2,3);
A=zeros(3,3,size(transforms,3)-2);
for i=1:(size(transforms,3)-2)
    p1(i,:)=transforms(1:3,4,i);
    p2(i,:)=transforms(1:3,4,i+1);
    C(i,:)=(p1(i,:)+p2(i,:))./2;
    d(i,1)=1.2*Distance(p1(i,:),p2(i,:));
    d(i,2)=1.2*Distance(p1(i,:),p2(i,:));
    d(i,3)=1.2*Distance(p1(i,:),p2(i,:));
    R(i,:)=[d(i,1)/2 d(i,2)/2 d(i,3)/2];
    A(1,1,i)=R(i,1)^-2;
    A(2,2,i)=R(i,2)^-2;
    A(3,3,i)=R(i,3)^-2;
    %hold on;

    %ellipsoid(C(i,1),C(i,2),C(i,3),R(i,1),R(i,2),R(i,3))

end

collision=false;
vertMat=vertcat(Objarray.transformedvertices);

for i=1:5:size(vertMat,1)
    
    for j=1:(size(transforms,3)-2)

        dist=(vertMat(i,:)-C(j,:))*A(:,:,j)*(vertMat(i,:)-C(j,:))';
        if (dist<=1)
            collision=true;
            break;
        end
    
    end
    if (collision==true)
        break;
    end
end
