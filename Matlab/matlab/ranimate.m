%parameters of the plot
axis(gca,'equal'); %the aspect ratio
axis([-5, 6, -1, 5]); %the limits
%紀錄移動點
x = 0;
y = 4;
center_x = 0; %圓心
for i=1:2000
    %畫車輪&車身
    cir = viscircles([center_x,0],1);
	lin = line([center_x, center_x+x],[0, y]);
    %當車身倒下時，停止運算
    if y>0
        x = 4*sin(out.phiscope.data(i));
        y = 4*cos(out.phiscope.data(i));
        center_x = out.thetascope.data(i)*pi/180 + sin(out.phiscope.data(i));
        t=i;
    end
    pause(0.00001);
    delete(cir);
    delete(lin);
end
t ; %看倒下幾毫秒;
center_x  %看倒下走多長公分(車輪中心)
%%
out.phiscope.data(1); %initial condition