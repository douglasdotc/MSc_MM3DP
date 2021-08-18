function RM= rm_clean(RM)
%% Note cleaning is actually bit useless...
disp("Should not use this actually")
idx=RM.voxValid;
RM.voxCenters=RM.voxCenters(idx,:);
RM.voxRIPoses=RM.voxRIPoses(idx);
RM.voxIKSuccess=RM.voxIKSuccess(idx);
RM.voxIKSolutions=RM.voxIKSolutions(idx);
RM.voxRI= RM.voxRI(idx);
end
