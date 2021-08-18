function [T,t]=gen_hcurve(density, scale, order,speed)
A = zeros(0,2);
B = zeros(0,2);
C = zeros(0,2);
D = zeros(0,2);

north = [ 0  1];
east  = [ 1  0];
south = [ 0 -1];
west  = [-1  0];


for n = 1:order
  AA = [B ; north ; A ; east  ; A ; south ; C];
  BB = [A ; east  ; B ; north ; B ; west  ; D];
  CC = [D ; west  ; C ; south ; C ; east  ; A];
  DD = [C ; south ; D ; west  ; D ; north ; B];

  A = AA;
  B = BB;
  C = CC;
  D = DD;
end

calc_len=@(T)sum(sqrt(sum(diff(T(:,1)).^2+diff(T(:,2)).^2,2)));

A = [0 0; cumsum(A)];
A=scale*A./max(A);
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