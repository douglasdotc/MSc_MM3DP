function ri= ri_zachprnn(RM)
n=size(RM.LocalRIPoses,1);
nn=7*n;
m=size(RM.voxValid,1);

TT=TForm.vec2tform(RM.LocalRIPoses);
mask=acosd(min(1,squeeze(abs(TT(3,3,:)))))<30 & squeeze(TT(3,3,:)<0);

ri=zeros(m,1);
for ii=1:m
    
     nnids=vind2nnind(RM,ii);
     nnids=nnids(RM.voxValid(nnids));     
     ri(ii)=100*sum(  cellfun(@(c) sum(c & mask) ,RM.voxIKSuccess(nnids) ))/nn;
    
end

end


function nnind = vind2nnind(RM,vind)
%% turn index of a voxel into indeces of nn
[vx,vy,vz]=ind2sub(size(RM.grid.X),vind);
nnxyz=[vx,vy,vz;
vx,vy,vz+1;
vx,vy,vz-1;
vx,vy+1,vz;
vx,vy-1,vz;
vx+1,vy,vz;
vx-1,vy,vz];
nnind=nan(7,1);
for ii=1:7
    try
        nnind(ii)=sub2ind(size(RM.grid.X),nnxyz(ii,1),nnxyz(ii,2),nnxyz(ii,3));
    catch
        
    end
end
nnind=rmmissing(nnind);

end

