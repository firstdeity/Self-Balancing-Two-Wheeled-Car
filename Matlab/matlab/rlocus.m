%%
A=[-0.231185976061678;-0.498866089249557;5.84393669821388;3.73021803318506;9.90763303129933];
B=[4.16554325826943;-0.893946443952200;43.7489661560407;792.115696320530;120.101219035737];
%%
k = (2*A(2)+1)*(B(1)+B(2))-A(1);
%算出矩陣ABCD
ssA = [0 1 0 0;
       (-1*A(1)*B(4))/k ((B(1)+B(2))*A(4)+A(1)*B(3))/k 0 (-1*A(1)*B(3)-(B(1)+B(2))*A(3))/k;
       0 0 0 1;
       ((2*A(2)+1)*B(4))/k ((-1*(2*A(2)+1)*B(3))-A(4))/k 0 ((2*A(2)+1)*B(3)+A(3))/k];
ssB = [0;((B(1)+B(2))*A(5)+(A(1)*B(5)))/k;0;((-1*(2*A(2)+1)*B(5))-A(5))/k];
ssC = [1 0 0 0;0 0 1 0];
ssD = [0;0];

s=tf('s');
I=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];

%用state space equation求出tranfer fuction
[t1,t2] = ss2tf(ssA,ssB,ssC,ssD);

%H = ssC*inv(s*I-ssA)*ssB + ssD
sys = minreal(ss(ssA,ssB,ssC,ssD));
rank(ctrb(ssA,ssB));
%t=0:0.01:3;
%H1=lsim(sys,zeros(length(t),1),t,[0 0 0.001 0]);
%plot(t,H1(:,1))
%%
sys = tf(1,t2);
rphi= tf(t1(1,:),t2)
rtheta= tf(t1(2,:),t2)
%H = ssC*inv(s*I-ssA)*ssB + ssD;
eig(sys)  %phi:49 -29 (17 -27)  theta:0 -8
phisys = tf(1,[1,-49.1369])*tf(1,[1,29.0586]);
thesys = tf(1,[1,-13.5872,0]);
rlocus(thesys);

%% phi PID
Kp1 = 10;
Ki1 = 100;
Kd1 = 0.01;
H = [1];
Gc = tf([Kd1,Kp1,Ki1],[1,0]);

Mc1 = tf(Gc*sys);
rlocus(Mc1);
%step(Mc);
%% theta PID
Kp2 = 0.9;
Ki2 = 0.01;
Kd2 = 0.133;

Gc2= tf([Kd2,Kp2,Ki2],[1,0]);
Mc2 = tf(Gc2*thesys);
rlocus(Mc2);
%step(Mc2);
%%
p=[-28.2+19.4j -28.2-19.4j -20+12.8j -20-12.8j];
K=place(ssA,ssB,p)
sys2 = ss(ssA-ssB*K,ssB,ssC,ssD);
H2=lsim(sys2,zeros(length(t),1),t,[0 0 0.001 0]);
plot(t,H2(:,1))
%eig(ssA-ssB*K);
%%
ssA1 = [0 0 0 0;
       0 0 0 (-1*A(1)*B(3)-(B(1)+B(2))*A(3))/k;
       0 0 0 1;
       0 0 0 ((2*A(2)+1)*B(3)+A(3))/k];
eig(ssA1)