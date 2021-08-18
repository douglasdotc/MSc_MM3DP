%% test:
 test_ind=1:(61^3);
 ind=InverseReachability.rm_point2ind(RM,RM.voxCenters(test_ind,:));
 
 all((ind-test_ind')==0) 
 all(all(abs(RM.voxCenters(ind,:)-RM.voxCenters(test_ind,:))<1e-5,2))