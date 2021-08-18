function animate_poses(poses,t,scale,axes)
% assumes poses 8 is time
r=quatrotate(quatinv(poses(:,4:end)),axes./norm(axes));
n=size(poses,1);
h = animatedline;
hold on
q=quiver3(poses(:,1),poses(:,2),poses(:,3),r(:,1),r(:,2),r(:,3),scale);
hold off
a=gca;
aa=[a.XLim a.YLim  a.ZLim];

xlabel('x')
ylabel('y')
zlabel('z')

tic
for k = 1:n
    addpoints(h,poses(k,1),poses(k,2),poses(k,3));    
    q.XData=poses(k,1);
    q.YData=poses(k,2);
    q.ZData=poses(k,3);
    q.UData=r(k,1);
    q.VData=r(k,2);
    q.WData=r(k,3);
    axis(aa);
    drawnow
    while toc<t(k)
    end
end


end



