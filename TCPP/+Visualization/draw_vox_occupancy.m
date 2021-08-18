function  draw_vox_occupancy(Vox,varargin)

[X, Y, Z]=ndgrid(Vox.spans{1},Vox.spans{2},Vox.spans{3});% expects centerpoints of cox to exist
N=sum(Vox.N,6);
N=sum(N,5);
N=sum(N,4);
n=N(N(:)>0);
p=[X(:),Y(:),Z(:)];
p=p(N(:)>0,:);

% c=n./max(n);
% c=-log(n-max(n));
% c=log(n./max(n));

if ~isempty(varargin)
    alpha=varargin{1};
else
    alpha=1;
end

if length(varargin)>1
    shape=varargin{2};
else
    shape='s';
end

if length(varargin)>2
    saza=varargin{3};
else
    saza=300;
end

c=n;
scatter3(p(:,1),p(:,2),p(:,3),saza,c,'filled',shape,'MarkerFaceAlpha',alpha,'MarkerEdgeAlpha',alpha);
colorbar
end

