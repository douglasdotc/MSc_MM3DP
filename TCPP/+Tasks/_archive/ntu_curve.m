function [T,t]=ntu_curve(density,speed)


% y = downsample(x,n)
% y = resample(x,p,q)

% Raw
% data=[   -0.2020    0.5240;
%   139.7960    0.3500;
%   171.2170  -18.2560;
%   181.6290   -9.0510;
%   145.2170   13.4110;
%    -5.0500   15.0190;
%   -42.4810   -8.2130;
%   -32.7300  -18.1920;
%    -0.2040    0.3500;
% ]

% cleaned
data=[   0    0;
  140    0;
  172  -18;
  180   -8;
  145   15;
   -5   15;
  -40   -8;
  -32  -18;
   0    0;
];
data=data./100;

%%%%%%%%%%%%%%%%%%%
calc_len=@(T)sum(sqrt(sum(diff(T(:,1)).^2+diff(T(:,2)).^2,2)));

A=data;
A = [0 0; (A)];
A_l=calc_len(A);

n = round(A_l*(1/density));
hcurve=interp1(1:size(A,1),A,linspace(1,size(A,1),n));
hcurve(:,3)=0;%Z set to zero

temp = atan2((hcurve(11:end,2)-hcurve(1:end-10,2)), (hcurve(11:end,1)-hcurve(1:end-10,1)));
yw = [temp(1:10);temp ];
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