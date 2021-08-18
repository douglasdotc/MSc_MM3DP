function RI= ri_zachpr(RM)
n=size(RM.LocalRIPoses,1);

TT=TForm.vec2tform(RM.LocalRIPoses);
mask=acosd(min(1,squeeze(abs(TT(3,3,:)))))<30 & squeeze(TT(3,3,:)<0);

RI=zeros(size(RM.voxValid,1),1);
RI(RM.voxValid)= vertcat(  cellfun(@(c) 100*sum(c & mask)/n ,{RM.voxIKSuccess{RM.voxValid}} ));

end

