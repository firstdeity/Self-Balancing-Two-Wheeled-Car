%clc;
%clear;

b = Bluetooth('u_t_chan',1);  %�إߤ@bluetooth����
fopen(b);

data_length = 1500;
data = zeros(data_length, 2);
%�s��phi,thetad
for i = 1:data_length
   data(i,1) = str2double(fscanf(b));
   data(i,2) = str2double(fscanf(b));
end

plot(data)
save(sprintf('%s\\%s\\position_1',pwd,''),'data');
fclose(b);
%save(sprintf('%s\\%s\\ID0_11data',pwd,''),'data'); %�s0pwm����
%save(sprintf('%s\\%s\\ID_10data',pwd,''),'data'); %�s50pwm����
%save(sprintf('%s\\%s\\balance',pwd,''),'data');
%% ��� ����
data3 = load("position.mat");  
data3 = data3.data;
phi = data3(:,1)/180*pi;       %�ন�|��
theta = data3(:,2);
%plot(data3);

o=downsample(out.phiscope.data,10);
hold on
plot(phi),grid;
plot(o);
legend("practical","simulation");
hold off