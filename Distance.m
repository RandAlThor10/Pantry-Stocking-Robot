function d=Distance(p1,p2)

x1=p1(1);
x2=p2(1);

y1=p1(2);
y2=p2(2);

z1=p1(3);
z2=p2(3);

d2=(x2-x1)^2+(y2-y1)^2+(z2-z1)^2;
d=sqrt(d2);

end