function sample = sample_vox_style_4d(radius,resolution,n)
%  this will put rotation to down and then apply n rotations on top
% Sample a cube
x=-radius:resolution:radius;
y=-radius:resolution:radius;
z=0;
[X,Y,Z] = meshgrid(x,y,z);
% Restrict cube to sphere
R=sqrt(X.^2+Y.^2+Z.^2);
X=X(R<=radius);
Y=Y(R<=radius);
Z=Z(R<=radius);
x=X(:);
y=Y(:);
z=Z(:);
m=length(X(:));
% gen snowflake poses
t=linspace(0,2*pi,n)';
amp=randn(n,1);
poses=TForm.vec2tform([zeros(length(t),3) eul2quat([cos(t).*amp,sin(t).*amp,t.*0],'XYZ')]);
down_tforms=[repmat(eul2rotm([0 pi 0],'XYZ'),1,1,n) zeros(3,1,n);zeros(1,3,n) ones(1,1,n)];
poses_tforms=TForm.tformX(down_tforms,poses);
poses=TForm.tform2vec(poses_tforms);

sample=zeros(n*m,7);

for ii=1:m    
    poses(:,1)=x(ii);
    poses(:,2)=y(ii);
    poses(:,3)=z(ii);
    sample(1+(ii-1)*n:ii*n,:)=poses;
end
% Visualization.draw_poses(TForm.tform2vec(poses_tforms),.1,[0 0 1],'b')


end

