data = load("ID0data.mat");  %�פJ���
data = data.data;
phi = data(:,2)/180*pi;
range = 1:150;    %����Q�����d��
o=downsample(out.phiscope(:,2),10);  %�פJsimulink�]�X�������ƾڡA�åB���C�����W�v�A�ϱo�Pdata���W�v�ۦP
hold on  %�e�b�P�@�i��
grid; 
plot(phi(range))
plot(o);
hold off