function poses = sample_cart_sphere_6dof(radius,resolution,n)
% poses= xyz qwqxqyqz
x=-radius:resolution:radius;
y=-radius:resolution:radius;
z=-radius:resolution:radius;


[X,Y,Z] = meshgrid(x,y,z);
R=sqrt(X.^2+Y.^2+Z.^2);
X=X(R<=radius);
Y=Y(R<=radius);
Z=Z(R<=radius);

m=length(X(:));
c=cell(m,1);

for ii=1:m
    c{ii}=[repmat([X(ii) Y(ii) Z(ii)],n,1) sampe_quart(n)];
end
poses=vertcat(c{:});

end


function qs=sampe_quart(n)
ss = rand(1,n);
s1 = sqrt(1-ss);
s2 = sqrt(ss);
t1 = 2*pi * rand(1,n);
t2 = 2*pi * rand(1,n);
w = cos(t2) .*s2;
x = sin(t1) .*s1;
y = cos(t1) .*s1;
z = sin(t2) .*s2;
qs=[w;x;y;z]';
end

