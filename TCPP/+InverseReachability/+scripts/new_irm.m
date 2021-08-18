%% Imports
import IK.*
import InverseReachability.*
%% %%%%%% Params %%%%%%
% load("RM_complete_Finally"); 

%% Computing RI Normal zacharias
% RM.voxRI= ri_zacharias(RM);
RM.voxRIz= ri_zacharias(RM);
RM.voxRIzpr= ri_zachpr(RM);
RM.voxRIzprnn= ri_zachprnn(RM);
%% Constructing IRM:

%NOTE I am not using RM to cosntruct, i assume I knwo how RM is
idx_rad = ceil(RM.params.gridrad ./ RM.params.gridres);
d = (-idx_rad:idx_rad) * RM.params.gridres;
M = length(d);
th=linspace(-pi,pi,73);
x=d;y=d;z=d(d>-.001);

IRM.gridres=[RM.params.gridres*ones(1,3) mean(diff(th))];

[X, Y, Z,TH] = ndgrid(x, y, z,th);
MM = size(X(:),1);

IRM.spans = {x,y,z,th};
IRM.grid.X = X;
IRM.grid.Y = Y;
IRM.grid.Z = Z;
IRM.grid.TH = TH;
%%
IRM.voxCenters = [X(:), Y(:), Z(:),TH(:)];
IRM.voxRIz = zeros(MM,1);
IRM.voxRIzpr = zeros(MM,1);
IRM.voxRIzprnn = zeros(MM,1);
%%
A=IRM.voxCenters;
A(:,3)=-A(:,3);
II=zeros(MM,1);
for ii=1:MM  
    R=eul2rotm([A(ii,4) 0 0],'ZYZ');     
    p=-R' *(A(ii,1:3)');     
    II(ii)=InverseReachability.rm_point2ind(RM,p');   
    ii/MM
end


for ii=1:MM
    
     if ~isnan(II(ii))
        IRM.voxRIz(ii)=RM.voxRIz(II(ii));
        IRM.voxRIzpr(ii)=RM.voxRIzpr(II(ii));
        IRM.voxRIzprnn(ii)=RM.voxRIzprnn(II(ii));
     end  
    ii/MM
end


%%
save("IRM_complete_final2","IRM")
