function IRM = irm_prune(IRM, prc)

    cutoff = prctile(IRM.voxRI(IRM.voxRI > 0), prc);
    IRM.voxRI(IRM.voxRI < cutoff) = 0;

end
