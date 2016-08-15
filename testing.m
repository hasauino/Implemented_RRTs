clear all
close all

r = 1;

h = 2;

theta = 0:0.05:2*pi;

x = r*cos(theta);
y = r*sin(theta);

y(end) = 0;

z1 = 0;

z2 = h;
col=[253/255 247/255 2/255];


set(gca,'NextPlot','Add');
h=patch(x,y,z2*ones(size(x)),'y');
set(h,'edgecolor','k','FaceColor',col)
patch(x,y,z1*ones(size(x)),'edgecolor','k','FaceColor',col);
surf([x;x],[y;y],[z1*ones(size(x));z2*ones(size(x))])
view(3)