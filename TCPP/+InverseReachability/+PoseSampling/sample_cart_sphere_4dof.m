function poses = sample_cart_sphere_4dof(radius,resolution,n)
% This function samples 4DOF poses specifically for youbot. XYZ free, then
% ZYZ rotation where Z1  is atan2, Y is to face EE down, Z2 is free sampled
% by n

% Sample a cube
x=-radius:resolution:radius;
y=-radius:resolution:radius;
z=-radius:resolution:radius;
[X,Y,Z] = meshgrid(x,y,z);

% Restrict cube to sphere
R=sqrt(X.^2+Y.^2+Z.^2);
X=X(R<=radius);
Y=Y(R<=radius);
Z=Z(R<=radius);
m=length(X(:));

%Create Poses
poses(:,1:3)=[X(:),Y(:),Z(:)];
% Comp X axis direction
zeul=atan2(Y(:),X(:));

%%
function sample_rotations()
temp_poses=cell(n,1);
zz=linspace(0,2*pi,n);%Always includes end points

for ii=1:n    
    temp_poses{ii}=[poses eul2quat([zeul,ones(m,1)*pi,ones(m,1)*zz(ii)],'ZYZ') ];    
end
poses=vertcat(temp_poses{:});

end
%%
sample_rotations()

end

