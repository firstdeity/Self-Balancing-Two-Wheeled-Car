clear;
clc;
%% 0pwm
data1 = load("ID0_12data.mat");  %�פJ�s��data 0pwm
data1 = data1.data;
phi = data1(:,2)/180*pi;       %�ন�|��
u = zeros(length(phi),1);

plot(phi),grid

range = 149:160;        %0pwm ���Ƚd�� 10:112:124;  12:149:160

%get phid
dt = 0.01; 
for i=1:length(phi)-1
    phid(i) = (phi(i+1)-phi(i))/dt;
end
phid(length(phi)) = phid(end);
phid = phid';

%get phidd
for i=1:length(phid)-1
    phidd(i) = (phid(i+1)-phid(i))/dt;
end
phidd(length(phid)) = phidd(end);
phidd = phidd';

%�p��Ufunction����
g2 = -2*cos(phi(range)).*phidd(range)+sin(2*phi(range)).*sec(phi(range)).*phid(range).*phid(range);
g4 = phid(range);
g8 = -phid(range);
g9 = sin(phi(range));
E = phidd(range);

%% u=0pwm�t�ΰѼ�
G1=[g2 g4];
G2=[g8 g9];
a1 = pinv(G1'*G1)*G1'*E
b1 = pinv(G2'*G2)*G2'*E

clear g2 g4 g8 g9 phi phid phidd range
%% 50pwm
data2 = load("ID_12data.mat");    %�פJ�s��data 50pwm
data2 = data2.data;
phi = data2(:,1)/180*pi;       %�ন�|��
thetad = data2(:,2);
u = zeros(length(phi),1);
for i=1:length(phi)
   u(i)=1;  %pwm 0or50
end
%for i=1:96
%    phi(i) = -0.006;
%end
plot(phi),grid;

range =37:48;         %50pwm ���Ƚd�� 10:58:63  12:37:48

%get thetadd
dt = 0.01;
for i=1:length(thetad)-1
    thetadd(i) = (thetad(i+1)-thetad(i))/dt;
end
thetadd(length(thetad)) = thetadd(end);
thetadd = thetadd';

%get phid
dt = 0.01; 
for i=1:length(phi)-1
    phid(i) = (phi(i+1)-phi(i))/dt;
end
phid(length(phi)) = phid(end);
phid = phid';

%get phidd
for i=1:length(phid)-1
    phidd(i) = (phid(i+1)-phid(i))/dt;
end
phidd(length(phid)) = phidd(end);
phidd = phidd';

%�p��Ufunction����
g1 = -thetadd(range);
g2 = -2*cos(phi(range)).*phidd(range)+sin(2.*phi(range)).*sec(phi(range)).*phid(range).*phid(range);
g3 = -thetad(range);
g4 = phid(range);
g5 = u(range);
g6 = -thetadd(range);
g7 = -cos(phi(range)).*thetadd(range);
g8 = thetad(range)-phid(range);
g9 = sin(phi(range));
g10 = -u(range);
E1 = phidd(range)-a1(1).*g2-a1(2).*g4;
E2 = phidd(range)-b1(1).*g8-b1(2).*g9;

%% u=50pwm�t�ΰѼ�
G3=[g1 g3 g5];
G4=[g6 g7 g10];
a2 = pinv(G3'*G3)*G3'*E1;
b2 = pinv(G4'*G4)*G4'*E2;

%��X
a = [a2(1);a1(1);a2(2);a1(2);a2(3)]
b = [b2(1);b2(2);b1(1);b1(2);b2(3)]
%A = [a2(1);a1(1);a2(2);a1(2);a2(3)];
%B = [b2(1);b2(2);b1(1);b1(2);b2(3)];