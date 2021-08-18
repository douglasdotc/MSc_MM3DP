function  scatter_vox(vox)
%Produces 3d scatter plot of the voxel structure. Takes only xyz vox 

 figure;
[X, Y, Z]=ndgrid(vox.xc,vox.yc,vox.zc);% expects centerpoints of cox to exist
 N=vox.N;% expects N to be the value
scatter3(X(N>0),Y(N>0),Z(N>0),20,N(N>0)./max(N(:)),'filled');
end

