function T = q2tform(q)
    % inmput Q, output Tform. with base up
    % output  nx3 vec. xyth OR nx4 vec xyth and t (from given)
    
    T=zeros(4,4,size(q,1));
    for ii=1:size(q,1)
        T(:,:,ii) = [eul2rotm([q.q(3) 0 0]) [q.q(1:2) 0]'; 0 0 0 1];
    end
    

end
