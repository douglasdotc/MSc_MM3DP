%% Docs
%  I have implemented a number of functions to help perform operations
%  between 3D arrays, vector forms of transforms. 
%% q2tform 
randtform=quat2tform(randrot)+ [zeros(4,3) [rand(3,1);0]]
assert(all(all(inv(randtform)-tform2inv(randtform)<1e-13)))