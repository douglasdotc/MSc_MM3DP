function [full_poses] = sample_cart_sphere_5dof(radius,resolution,n)
% This function samples 5DOF poses specifically for youbot. XYZ free, then
% ZYZ rotation where Z1  is atan2, Y and Z2 is free sampled
% by n. sampled on 0:2pi tho. So not perfect. 
%Poses= xyz qwqxqyqz
x=-radius:resolution:radius;
y=-radius:resolution:radius;
z=-radius:resolution:radius;
[X,Y,Z] = meshgrid(x,y,z);

R=sqrt(X.^2+Y.^2+Z.^2);
X=X(R<=radius);
Y=Y(R<=radius);
Z=Z(R<=radius);

poses(:,1:3)=[X(:),Y(:),Z(:)];
zeul=atan2(Y(:),X(:));
full_poses= spawn_poses(poses,zeul,n);

end

function poses=spawn_poses(poses,zeul,n)
% 
temp_poses=cell(n*n,1);
m=size(poses,1);
zz=linspace(0,2*pi,n);%Always includes end points
yy=linspace(0,2*pi,n);%Always includes end points

for ii=1:n
    for jj=1:n        
        temp_poses{(ii-1)*n+jj}=[poses eul2quat([zeul,ones(m,1)*yy(ii),ones(m,1)*zz(jj)],'ZYZ') ];    
    end  
end
poses=vertcat(temp_poses{:});

end


