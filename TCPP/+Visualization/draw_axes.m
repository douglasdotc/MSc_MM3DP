function draw_axes(x,y,z,quat,scale,a)
% This creates xyz axes  ar points xyz and quat  pose.    
    ax=quat2rotm(quat)*scale;
    if a(1) quiver3(x,y,z,ax(1,1),ax(2,1),ax(3,1),'r'); end
    if a(2) quiver3(x,y,z,ax(1,2),ax(2,2),ax(3,2),'g'); end
    if a(3) quiver3(x,y,z,ax(1,3),ax(2,3),ax(3,3),'b'); end
    drawnow
end
