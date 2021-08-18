%% test:


MM = size(IRM.voxCenters, 1);
test_ind = ceil(rand(10000, 1) * MM);
% test_ind = 1:100;
ind = InverseReachability.irm_point2ind(IRM, IRM.voxCenters(test_ind, :));

all((ind - test_ind) == 0)
all(all(abs(IRM.voxCenters(ind, :) - IRM.voxCenters(test_ind, :)) < 1e-5, 2))
