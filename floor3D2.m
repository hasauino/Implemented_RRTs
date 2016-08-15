%% RRT* implementation
% Hassan Umari
% reference: Sampling-based Algorithms for Optimal Motion Planning,
% Emilio Frazzoli,Sertac Karaman
%Page 16
%%
close all
clear all
clc
xdim=100;
ydim=100;
zdim=50;
fig=figure('name','Received Data','units','normalized','position',[0.29 0.06 0.7 .83]);
ax=gca; backColor=[222/255 222/255 239/255];
ax.Color = backColor;
axis([0 xdim 0 ydim 0 zdim])
grid

% %% Floor
% [Xfloor,Yfloor] = meshgrid(0:xdim/2:xdim,0:ydim/2:ydim);    Zfloor=100*zeros(size(Yfloor,1),size(Yfloor,2));
% groundSurf=surface(Xfloor,Yfloor,Zfloor,'LineStyle','none');
% texture = imread('1.jpg'); %import the image.
% groundSurf.FaceColor = 'texturemap'; %Tell the surface to use a texturemap for coloring
% groundSurf.CData = texture; %Now set the color data of the surface to be the imported image data.
%% Camera settings
%view(3)
fig.Children.Projection = 'perspective';
camva(5);
origTarget = [50,45,7];
camtarget(origTarget);
material('dull') %Not really needed, just makes the plane shinier.
camlight('headlight'); %... and better lit.
%%
global eta d gama
eta=3; %growth distance (between nearset point towards the random point)
d=2; %2D space
gama=200; %check this????????????????????????
%% declaring obtacles (as points)
% x1 --> x2   y1-->y2    ob#=[x1 x2 y1 y2]
global obd
obd=5;
extrude=50;
ob1=[0 20 95 100];plotObstacleExtrude(ob1);
ob2=[80 100  0  2];plotObstacleExtrude(ob2);
ob3=[40 60  30  40];plotObstacleExtrude(ob3);
ob4=[30 40  0  40];plotObstacleExtrude(ob4);
ob5=[0 20 80 85];plotObstacleExtrude(ob5);
ob6=[80 88  18  20];plotObstacleExtrude(ob6);
ob7=[80 82  2  18];plotObstacleExtrude(ob7);
ob8=[92 100  18  20];plotObstacleExtrude(ob8);
ob9=[98 100  2  18];plotObstacleExtrude(ob9);
%ob10=[40 50  50  60];plotObstacleExtrude(ob10);
ob11=[50 60  70  80];plotObstacleExtrude(ob11);
ob12=[0 10  50  60];plotObstacleExtrude(ob12);
obstacles=[ob1;ob2;ob3;ob4;ob5;ob6;ob7;ob8;ob9;ob11;ob12];
%% Target
Target=[30 80];radius=5; circle(Target(1),Target(2),radius);
% x1 = 28;y1 = 82.25;str1 = 'Target';text(x1,y1,str1,'Color','k','FontSize',8);
circleExtrude(30,80,5);
%% obtaining the random inital point
z=0;
while(z==0)
 
 V(1,:)=[xdim*rand ydim*rand];
 z=ObtacleFree(V(1,:),obstacles);
end
%V(1,:)=[35 25];   uncomment to force inital position
V(1,:)=[90,30];
plot(V(1,1), V(1,2), '.k', 'MarkerSize',30)
%%


Cost(1,:)=[V 0];
vv=[V V];

iter=0; %loop counter,keeps track of the number of iterations 
stp=0; %%stop condition initally zero, when it reaches target it is set to 1
points=[];
inc=0;
while (iter<5) 
   
iter=iter+inc;

xrand=[xdim*rand ydim*rand];        %%pdf_Fazzoli_p16______________line3
xnearest=Nearest(xrand,V);          %%pdf_Fazzoli_p16______________line4
xnew=Steer(xnearest,xrand);         %%pdf_Fazzoli_p16______________line5

if CollisionFree(xnearest,xnew,obstacles)%pdf_Fazzoli_p16__________line6
V=[V;xnew];
temp=(gama*log(length(V))/length(V))^(1/d);
r=min(eta,temp);r=eta*2;
Xnear=Near(V,xnew,r);
xmin=xnearest;


costrow=find(any(Cost(:,1)==xnearest(1),2));
cost=Cost(costrow,3)+norm(xnearest-xnew);
cmin=cost;
Cost=[Cost;[xnew cost]];
oldc=cost;


for indx=1:size(Xnear,1)
xnear=Xnear(indx,:);
costrow=find(any(Cost(:,1)==xnear(1),2));
cxnear=Cost(costrow,3)+norm(xnear-xnew);

if CollisionFree(xnear,xnew,obstacles) &&  (cxnear<cmin)
 xmin=xnear; cmin=cxnear; 

end

end
costrow=find(any(Cost(:,1)==xnew(1),2));
Cost(costrow,:)=[]; 


costrow=find(any(Cost(:,1)==xmin(1),2));
cost=Cost(costrow,3)+norm(xmin-xnew); 
Cost=[Cost;[xnew cost]];
vv=[vv;[xmin xnew]];
plot([xmin(1) xnew(1)],[xmin(2) xnew(2)],'b') %% plot tree branch

%% Rewire the tree
for indx=1:size(Xnear,1)
    xnear=Xnear(indx,:);
   cost=cost+norm(xnew-xnear); 
   costrow=find(any(Cost(:,1)==xnear(1),2));
  
   if CollisionFree(xnew,xnear,obstacles) &&  (cost<Cost(costrow,3))
 row=find(any(vv(:,3)==xnear(1),2));
 
 plot([vv(row,1) xnear(1)],[vv(row,2) xnear(2)],'Color',backColor)%delete old branch on the plot
 vv(row,:)=[];%delete old branch from the edges matrix
 vv=[vv;[xnew xnear]];

 plot([xnew(1) xnear(1)],[xnew(2) xnear(2)],'b')%draw the new branch

end
   
   
end




%%
if norm(xnew-Target)<radius
    stp=stp+1;
    inc=1;
    points=[points;xnew]; 
end

drawnow update 
end
end
cmin=inf;

for zz=1:size(points,1)
       cr=find(any(Cost(:,1)==points(zz,1),2)); %cost row
       if Cost(cr,3)<cmin
           cmin=Cost(cr,3);
           crowmin=zz;
       end
end
row=find(any(vv(:,3)==points(crowmin,1),2));pathpoints=[vv(row,3) vv(row,4)];
while row>1
        parentx=vv(row,1);
        plot([vv(row,1) vv(row,3)],[vv(row,2) vv(row,4)],':r','LineWidth',3);
        pathpoints=[pathpoints;[vv(row,1) vv(row,2)]];
        row=find(any(vv(:,3)==parentx,2));
        
    end
pathpoints=[pathpoints;[V(1,1) V(1,2)]];

%% increasing number of points on the path
pat=[];
for i=1:(size(pathpoints,1)-1)
    n=eta*5;
    r=norm(pathpoints(i,:)-pathpoints(i+1,:))/n;
    
    for j=1:n
    pat=[pat;Steer2(pathpoints(i,:),pathpoints(i+1,:),j*r)];

    end
end

plot(pat(:,1),pat(:,2),':r','LineWidth',3)


pause(2);
%% Patch, robot follows path
global orig 
patchData = stlread('robot.stl'); %THIS IS THE ONLY LINE THAT'S REALLY DIFFERENT FROM BEFORE.
p1 = patch(patchData,'EdgeColor','none','FaceColor','[.2 0.5 1.0]');
orig=p1.Vertices;
iter=1;
%% smoothing the path
ax=pat(1,1);
ay=pat(1,2);

pats=[];
for t=1:size(pat,1)
    f=0.98;
    ax=f*ax+(1-f)*pat(t,1);
    ay=f*ay+(1-f)*pat(t,2);
    pats=[pats;[ax ay]];
end
%% Follow smoothed path
 plot(pats(:,1),pats(:,2),'y','LineWidth',2)
 for k=size(pat,1):-1:2

     xdiff=pats(k-1,2)-pats(k,2);
     ydiff=pats(k-1,1)-pats(k,1);
 
     th=atan2(ydiff,xdiff);
     camx=40*sin(th);
     camy=40*cos(th);
    
 origTarget = [pats(k-1,1),pats(k-1,2),2];origPos = [pats(k-1,1)-camx,pats(k-1,2)-camy,10]';
campos(origPos);
camtarget(origTarget);
     
     camva(30);
     
    move2(p1,pats(k,1),pats(k,2),0,th);
    pause(.01)
    iter=iter+1;
 end
pause(1)
%% change camer view at the end
for i=1:100
    posx=((100-origPos(1))/100)*i+origPos(1);
    posy=((-90-origPos(2))/100)*i+origPos(2);
    posz=((50-origPos(3))/100)*i+origPos(3);
campos([posx posy posz]);
pause(0.02)
end