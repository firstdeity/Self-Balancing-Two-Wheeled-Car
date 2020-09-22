data = load("ID0data.mat");  %匯入資料
data = data.data;
phi = data(:,2)/180*pi;
range = 1:150;    %選取想取的範圍
o=downsample(out.phiscope(:,2),10);  %匯入simulink跑出的模擬數據，並且降低取樣頻率，使得與data的頻率相同
hold on  %畫在同一張圖
grid; 
plot(phi(range))
plot(o);
hold off