function  viz_manip(Vox,fun)

[X, Y, Z]=ndgrid(Vox.spans{1},Vox.spans{2},Vox.spans{3});% expects centerpoints of cox to exist
N=sum(Vox.N,6);
N=sum(N,5);
N=sum(N,4);
n=N(N(:)>0);
p=[X(:),Y(:),Z(:)];
p=p(N(:)>0,:);
c=fun(n);
subplot(2,1,1)
scatter3(p(:,1),p(:,2),p(:,3),20,c,'filled','MarkerFaceAlpha',.7,'MarkerEdgeAlpha',.7);
colorbar
subplot(2,1,2)
histogram(c)
end

