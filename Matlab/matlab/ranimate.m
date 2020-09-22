%parameters of the plot
axis(gca,'equal'); %the aspect ratio
axis([-5, 6, -1, 5]); %the limits
%���������I
x = 0;
y = 4;
center_x = 0; %���
for i=1:2000
    %�e����&����
    cir = viscircles([center_x,0],1);
	lin = line([center_x, center_x+x],[0, y]);
    %�����ˤU�ɡA����B��
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
t ; %�ݭˤU�X�@��;
center_x  %�ݭˤU���h������(��������)
%%
out.phiscope.data(1); %initial condition