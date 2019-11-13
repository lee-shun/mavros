clear all;
clc;
figure;
%%%%%%可设的基本参数%%%%%
global r;
NUAV=6;%%给出天上盘旋的无人机数量
Npoint=6;%%给出目标点数量
v=10;%给出飞行速度
r=30;%给出盘旋半径
g=9.8;
hmin=30;%给出飞机最小盘旋高度
h=5;%给出飞机高度间隔
H=[hmin:h:hmin+h*NUAV-h]';%生成每架飞机高度列表
%计算每架飞机对应的平抛距离
for i=1:NUAV
    paolenth(i)=v*sqrt(2*H(i)/g);
end
%%%%生成场地和目标点坐标,这是侦查组会给我的信息%%%%%%%
P=rand(Npoint,2);
P(:,1)=P(:,1)*100-50;
P(:,2)=P(:,2)*300-150;
P=[P zeros(Npoint,1)];
plot3(P(:,1),P(:,2),P(:,3),'r.');%画出目标点
hold on;
grid;
%%%把目标点按离圆心的距离排序,先排够打一轮的%%%
for i=1:NUAV
    for j=1:Npoint
        s=sqrt(P(j,1)^2+P(j,2)^2);%找出离圆心最近的点
        if j==1
            s0=s;
            n=1;
        end
        if s<s0
            s0=s;
            n=j;%目标点n为飞机i的攻击点，水平与i距离s0
        end
    end
        Prank(i,1)=P(n,1);
        Prank(i,2)=P(n,2);
        P(n,1)=10000;P(n,2)=10000;%把找过的好点弄出去
        Prank(i,3)=H(i);%不降高的投掷点
        pointlenth(i)=s0;%储存点到圆心的距离
end
%%%%%%%%%%%%%%%%生成飞机盘旋轨道%%%%%%%%
for i=1:NUAV
theta=0:pi/1800:2*pi;
Circlex=r*cos(theta);
Circley=r*sin(theta);
Circlez=H(i)*ones(1,3601);
plot3(Circlex,Circley,Circlez,'m','Linewidth',1);
hold on;
end
%%%%%%%%%%飞机逆时针飞行%%%%%%%%%
%%%@@@@@@@@@@@起飞模式切盘旋的航点是（r,0,H(i)），避免相撞从最上层开始盘
%%%%%%%%%画路径%%%%%
Ptou=zeros(NUAV,2);%生成投放点位置储存矩阵
Ptou=[Ptou H];

for i=1:NUAV
    R=sqrt(r^2+paolenth(i)^2);
     goal=[Prank(i,1) Prank(i,2)];
    if pointlenth(i)>=R%%%%%目标点在圆外且切线长足够平抛%%%%%%%
       tan=point_tangency(goal);
       point1=tan;%
       point2=goal-(goal-tan)/norm(goal-tan)*paolenth(i);
       plot3(point1(1),point1(2),H(i),'gO');%画point1(绿o）,在这一点盘旋转平飞
        plot3(point2(1),point2(2),H(i),'g*');%画point2(绿*),在这一点释放沙包、平飞切降落
        plot3( [point1(1),goal(1)],[point1(2),goal(2)], [H(i),H(i)],'g-'); 
        hold on;
    end



    if pointlenth(i)<R&&pointlenth(i)>=r%%%%目标点在圆外然而切线长不够平抛%%%%
        tan=point_tangency(goal);
        tanlenth=norm(tan-goal);%计算范数

%         Ptangency(i,1)=-tan(1);
%         Ptangency(i,2)=-tan(2);

        point1=-tan;
        aaa=(paolenth(i)-tanlenth)*(tan-goal)/tanlenth;
        point2=point1+aaa;
        point3=tan+aaa;
        plot3(point1(1),point1(2),H(i),'bo');%画point1（蓝+，切点的中心对称点），在这一点盘旋转平飞
        plot3(point2(1),point2(2),H(i),'b+');%画point2（蓝+）在这一点平飞转盘旋
        plot3(point3(1),point3(2),H(i),'b*');%画point3（蓝+）在这一点释放沙包、平飞切降落
        plot3( [point1(1),point2(1)],[point1(2),point2(2)], [H(i),H(i)],'b-');
        plot3( [point3(1),goal(1)],[point3(2),goal(2)], [H(i),H(i)],'b-');
        hold on;
    end



    if pointlenth(i)<r
        if pointlenth(i)==0
            goal=goal+0.0000001;
        end
        point1=-goal/pointlenth(i)*r;
        point1=[point1 0];
        w=[0 0 1];
        bbb=cross(w,point1);
        bbb=[bbb(1) bbb(2)];
        point1=[point1(1) point1(2)];
        point2=point1+bbb;
        point3=2*bbb;
        point5=goal/pointlenth(i)*(pointlenth(i)+paolenth(i));
        point4=point3+point5;
        plot3(point1(1),point1(2),H(i),'mo');%画point1（粉*），在这一点盘旋转平飞
        plot3(point2(1),point2(2),H(i),'m+');%画point2（粉*），在这一点平飞转盘旋
        plot3(point3(1),point3(2),H(i),'mo');%画point3（粉*），在这一点盘旋转平飞
        plot3(point4(1),point4(2),H(i),'m+');%画point4（粉*），在这一点平飞转盘旋
        plot3(point5(1),point5(2),H(i),'m*');%画point5（粉*），在这一点释放沙包、平飞切降落
        plot3( [point1(1),point2(1)],[point1(2),point2(2)], [H(i),H(i)],'r--');
        plot3( [point3(1),point4(1)],[point3(2),point4(2)], [H(i),H(i)],'r--');
        plot3( [point5(1),goal(1)],[point5(2),goal(2)], [H(i),H(i)],'r--');
        hold on;
    end
end
axis equal;
