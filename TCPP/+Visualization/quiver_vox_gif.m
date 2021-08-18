function  quiver_vox_gif(vox)
figure('Color',[1 1 1]);
for ii=1:size(vox.PoseGrid,3)
    p=vertcat(vox.PoseGrid{:,:,ii,:});
    t=zeros(4,4,size(p,1));
    for jj=1:size(p,1)
    t(:,:,jj)=[quat2rotm(p(jj,[7 4:6])) p(jj,1:3)'; [0 0 0 1]];
    end
    quiver(t(1,4,:),t(2,4,:),t(1,1,:),t(2,1,:),0.9)
    view(2)
    title(sprintf("z=%f",vox.zc(ii)));
    drawnow;
    pause(1);
end
end

