function [T,t]=ntu_mush(density,speed)
% divisions=8;
width=0.06/2;
length=0.2;

%%%
ntu=[ 
  180   -8;  
  145   15;
   -5   15;
  -40   -8;  
];
ntu=flipud(ntu);
ntu=ntu./100;
calc_len=@(T)sum(sqrt(sum(diff(T(:,1)).^2+diff(T(:,2)).^2,2)));
A=ntu;
A = [(A)];
A_l=calc_len(A);
n = round(A_l*(1/(density/10)));
ntucurve=interp1(1:size(A,1),A,linspace(1,size(A,1),n));
temp = atan2((ntucurve(11:end,2)-ntucurve(1:end-10,2)), (ntucurve(11:end,1)-ntucurve(1:end-10,1)));
yw = [temp(1:10);temp ];
ntucurve(:,3)=yw;
ntucurve(:,3)=0;

ntucurve=[ntucurve zeros(size(ntucurve,1),1)];
for ii=1:size(ntucurve,1)
    ntucurve(ii,4)=calc_len(ntucurve(1:ii,1:2));
end

ntucurve_func=@(t)interp1(ntucurve(:,4),ntucurve(:,1:3),t);

%%%%%%%%%%%%%%%%%%%%%%%%
% Mush shape
% shape=[0 0 0;
% -1 0 1;
% -1 -1 2;
% 1  -1 3;
% 1 0 4;
% 0 0 5;
% 0 1 6;
% 2 1 7;
% 2 0 8;
% 0 0 9];

shape=[0 1 ;
    1 1;
    1 0;
    0 0;
    0 -1;
    2 -1;
    2 0;
    1 0;    
    1 1;
    2 1];
    

shape(:,1)=length*shape(:,1);
shape(:,1)=shape(:,1)/2;
% shape(:,1)=shape(:,1)*0;
shape(:,2)=width*shape(:,2);

calc_len=@(T)sum(sqrt(sum(diff(T(:,1)).^2+diff(T(:,2)).^2,2)));

for ii=1:10
    shape(ii,3)=calc_len(shape(1:ii,1:2));
end
% mush=interp1(shape(:,3),shape(:,1:2),t);
mush_func=@(t)interp1(length*shape(:,3)./shape(end,3),shape(:,1:2),t);
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
points=zeros(n,3);
for ii=1:n
    pp=mush_func(mod((ii/n)*A_l,length));
    prog=pp(1); 
    pp(1)=0;
    
    p=ntucurve_func(min(prog+(ii/n)*A_l-mod((ii/n)*A_l,length),A_l));
    
    T=[cos(p(3)) -sin(p(3)) p(1);sin(p(3)) cos(p(3)) p(2); 0 0 1];
%     points(ii,:)=(T*[mush_func(mod(ii./n,length)) 1]')';
    points(ii,:)=(T*[pp 1]')';
%     plot(points(1:ii,1),points(1:ii,2))
end
points=points(~isnan(points(:,1)),:);
% plot(points(:,1),points(:,2))
% axis equal
% hold off
% 
% mm=ntucurve_func(linspace(0,A_l,10));
% hold on
% plot(mm(:,1),mm(:,2),'.')
% axis equal
% hold off
%%


%%%%%%%%%%
% 
A=points(:,1:2);

A_l=calc_len(A);

n = round(A_l*(1/density));
hcurve=interp1(1:size(A,1),A,linspace(1,size(A,1),n));
hcurve(:,3)=0;%Z set to zero

temp = atan2((hcurve(11:end,2)-hcurve(1:end-10,2)), (hcurve(11:end,1)-hcurve(1:end-10,1)));
yw = [temp(1:10);temp ];
% yw(:)=0;
q=eul2quat([ones(size(yw))*pi,ones(size(yw))*0.0,-yw],'XYZ');% Would be good to double check this from time to time.
hcurve(:,4:7)=q;

% Set time stamps
t=[0;cumsum(sqrt(sum(diff(hcurve(:,1:3)).^2,2)))/speed];
hcurve(:,8)=t;
disp("Curve generated, length:");
disp(calc_len(hcurve));
T=vec2tform(hcurve(:,1:7));
t=hcurve(:,8);

end