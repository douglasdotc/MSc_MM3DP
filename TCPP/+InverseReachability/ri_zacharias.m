function RI= ri_zacharias(RM)
n=size(RM.LocalRIPoses,1);
RI= vertcat(cellfun(@(c) 100*sum(c)/n ,RM.voxIKSuccess ));
end

