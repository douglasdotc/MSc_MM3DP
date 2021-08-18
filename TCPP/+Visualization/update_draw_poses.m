function update_draw_poses(qh,poses,axes)


r=quatrotate(quatinv(poses(:,4:end)),axes./norm(axes));
poses(:,1:3)=round(poses(:,1:3),8);
r=round(r,8);
qh.XData=poses(:,1);
qh.YData=poses(:,2);
qh.ZData=poses(:,3);

qh.UData=r(:,1);
qh.VData=r(:,2);
qh.WData=r(:,3);


end

