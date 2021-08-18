%% Docs
These functions extend tform functionality:
%% tformX multiplies tform 4x4xn array with 4x4xn or 4x4
%%  tform2inv inverses all tforms in a 4x4xn array

%% docs
% - The functions  support  passing preallocated inputs but this does not really influence  behjaviour with the exception of some wierd glitch at n =100
%% tform2vec transform tform to vec form
n=100000
T=rand(4,4,n);
vec=zeros(n,7);
assert(isequal(size(tform2vec(T)), [n,7]))
timeit(@() tform2vec(T))
assert(isequal(size(tform2vec(T,vec)), [n,7]))
timeit(@() tform2vec(T,vec))

%% vec2tform transform vec 2 tform
n=100000
vec=[rand(n,3) compact(randrot(n,1))];
T=zeros(4,4,n);
assert(isequal(size(vec2tform(vec)), [4,4,n]))
timeit(@()vec2tform(vec))
assert(isequal(size(vec2tform(vec,T)), [4,4,n]))
timeit(@()vec2tform(vec,T))
