function ind=rm_point2ind(RM,points)
    %given rm, convert point to index
    % Assert that grid is equidistant
    
    % Note this can be made faster without for loop

    assert(size(points,2)==3);
    n=size(points,1);
    r=RM.params.gridres/2;    
    ind=zeros(n,1);
    for ii=1:n
        xid=find( abs( points(ii,1)-RM.spans{1}   )<=r,1);
        yid=find( abs( points(ii,2)-RM.spans{2}   )<=r,1);
        zid=find( abs( points(ii,3)-RM.spans{3}   )<=r,1);

        if ~isempty(xid) && ~isempty(yid) && ~isempty(zid)    
            %%TODO THIS IS BONKERS
            ind(ii)=sub2ind(size(RM.grid.X),yid,xid,zid);
        else
            ind(ii)=nan;
            disp("rm_point2ind: point not in RM")
        end
    end
    
    
end


% 