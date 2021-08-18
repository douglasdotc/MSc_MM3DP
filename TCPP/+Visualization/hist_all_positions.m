function hist_all_positions(poses)
[N,c] = hist3(poses(:,1:2),'Nbins',[50 50]);
figure
zu=unique(poses(:,3));
hz = histogram(poses(:,3),min(10,length( zu([diff(zu)>0.01]) )));
dz=diff(hz.BinEdges)/2; dz=dz(1);
zc=hz.BinEdges(1:end-1)+diff(hz.BinEdges)/2;

for ii=1:length(zc)
    z=zc(ii);
    subplot(2,ceil(length(zc)/2),ii)
    if sum(abs(poses(:,3)-z)<dz)>0        
        hist3(poses(abs(poses(:,3)-z)<dz,1:2),'CdataMode','auto','FaceAlpha',1,'Ctrs',c,'EdgeColor','none');    
        colorbar;
    end
    title(sprintf("z=%f",z));
    view(2)
    drawnow
    pause(1)    
end
end