clear all;
clc;
figure;
%%%%%%�����Ļ�������%%%%%
global r;
NUAV=6;%%�����������������˻����
Npoint=6;%%����Ŀ�������
v=10;%���������ٶ�
r=30;%���������뾶
g=9.8;
hmin=30;%�����ɻ���С�����߶�
h=5;%�����ɻ��߶ȼ���
H=[hmin:h:hmin+h*NUAV-h]';%����ÿ�ܷɻ��߶��б�
%����ÿ�ܷɻ���Ӧ��ƽ�׾���
for i=1:NUAV
    paolenth(i)=v*sqrt(2*H(i)/g);
end
%%%%���ɳ��غ�Ŀ��������,���������������ҵ���Ϣ%%%%%%%
P=rand(Npoint,2);
P(:,1)=P(:,1)*100-50;
P(:,2)=P(:,2)*300-150;
P=[P zeros(Npoint,1)];
plot3(P(:,1),P(:,2),P(:,3),'r.');%����Ŀ����
hold on;
grid;
%%%��Ŀ���㰴��Բ�ĵľ�������,���Ź���һ�ֵ�%%%
for i=1:NUAV
    for j=1:Npoint
        s=sqrt(P(j,1)^2+P(j,2)^2);%�ҳ���Բ�������ĵ�
        if j==1
            s0=s;
            n=1;
        end
        if s<s0
            s0=s;
            n=j;%Ŀ����nΪ�ɻ�i�Ĺ����㣬ˮƽ��i����s0
        end
    end
        Prank(i,1)=P(n,1);
        Prank(i,2)=P(n,2);
        P(n,1)=10000;P(n,2)=10000;%���ҹ��ĺõ�Ū��ȥ
        Prank(i,3)=H(i);%�����ߵ�Ͷ���
        pointlenth(i)=s0;%�����㵽Բ�ĵľ���
end
%%%%%%%%%%%%%%%%���ɷɻ���������%%%%%%%%
for i=1:NUAV
theta=0:pi/1800:2*pi;
Circlex=r*cos(theta);
Circley=r*sin(theta);
Circlez=H(i)*ones(1,3601);
plot3(Circlex,Circley,Circlez,'m','Linewidth',1);
hold on;
end
%%%%%%%%%%�ɻ���ʱ������%%%%%%%%%
%%%@@@@@@@@@@@����ģʽ�������ĺ����ǣ�r,0,H(i)����������ײ�����ϲ㿪ʼ��
%%%%%%%%%��·��%%%%%
Ptou=zeros(NUAV,2);%����Ͷ�ŵ�λ�ô�������
Ptou=[Ptou H];
for i=1:NUAV
    R=sqrt(r^2+paolenth(i)^2);
     goal=[Prank(i,1) Prank(i,2)];
    if pointlenth(i)>=R%%%%%Ŀ������Բ�������߳��㹻ƽ��%%%%%%%
       tan=point_tangency(goal);
       point1=tan;
       point2=goal-(goal-tan)/norm(goal-tan)*paolenth(i);
       plot3(point1(1),point1(2),H(i),'go');%��point1(��o��,����һ������תƽ��
        plot3(point2(1),point2(2),H(i),'g*');%��point2(��*),����һ���ͷ�ɳ����ƽ���н���
        plot3( [point1(1),goal(1)],[point1(2),goal(2)], [H(i),H(i)],'g-'); 
        hold on;
    end
    if pointlenth(i)<R&&pointlenth(i)>=r%%%%Ŀ������Բ��Ȼ�����߳�����ƽ��%%%%
        tan=point_tangency(goal);
        tanlenth=norm(tan-goal);
%         Ptangency(i,1)=-tan(1);
%         Ptangency(i,2)=-tan(2);
        point1=-tan;
        aaa=(paolenth(i)-tanlenth)*(tan-goal)/tanlenth;
        point2=point1+aaa;
        point3=tan+aaa;
        plot3(point1(1),point1(2),H(i),'bo');%��point1���+���е������ĶԳƵ㣩������һ������תƽ��
        plot3(point2(1),point2(2),H(i),'b+');%��point2���+������һ��ƽ��ת����
        plot3(point3(1),point3(2),H(i),'b*');%��point3���+������һ���ͷ�ɳ����ƽ���н���
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
        plot3(point1(1),point1(2),H(i),'mo');%��point1����*��������һ������תƽ��
        plot3(point2(1),point2(2),H(i),'m+');%��point2����*��������һ��ƽ��ת����
        plot3(point3(1),point3(2),H(i),'mo');%��point3����*��������һ������תƽ��
        plot3(point4(1),point4(2),H(i),'m+');%��point4����*��������һ��ƽ��ת����
        plot3(point5(1),point5(2),H(i),'m*');%��point5����*��������һ���ͷ�ɳ����ƽ���н���
        plot3( [point1(1),point2(1)],[point1(2),point2(2)], [H(i),H(i)],'r--');
        plot3( [point3(1),point4(1)],[point3(2),point4(2)], [H(i),H(i)],'r--');
        plot3( [point5(1),goal(1)],[point5(2),goal(2)], [H(i),H(i)],'r--');
        hold on;
    end
end

axis equal;
