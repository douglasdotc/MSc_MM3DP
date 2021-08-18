ypik = IK.YPIKObj();
%%

load("FloorTask")
load("FloorplanBest")
%%

b=path(1).qmat(path');
b(:,3)=unwrap(b(:,3));
b=interp1(b(:,4),b(:,1:3),linspace(0,1,size(T,3)));
assert(~any(isnan(b(:))))
B=pathmat2tforms(b);

A=TForm.tformX(TForm.tform2inv(B),T)  ;
A=TForm.tformX(A,TForm.DOWN);

tp=squeeze(T(1:3,4,:))';
ll=cumsum(  sqrt(sum(diff(tp).^2 ,2) ) );

ll=cumsum(  sqrt(sum(diff(b(:,1:2)).^2 ,2) ) );


ll=cumsum(  sqrt(sum(diff(b(:,3)).^2 ,2) ) )


a=squeeze(A(1:3,4,:))';
inds=+InverseReachability.rm_point2ind(RM,a)

avals=RM.voxRIzprnn(inds)

RM.voxRIzprnn=RM.voxRIzprnn(RM.voxRIzprnn>0)
mean(RM.voxRIzprnn(inds))/ 0.0700
 6.2143- 90th

%%
ids=500;
ide=1500;
nn=ide-ids +1;

A=A(:,:,ids:ide);
n=size(A,3);
poses=TForm.tform2vec(A)
 default_settings = {"use_last", true, "timeout", 0.05, "attempts", 20};


ik_result = ypik.get_ik(TForm.tform2vec(A), "base_footprint", default_settings{:})
  joint_tr = zeros(n, 10);

    for ii = 1:length(ik_result.sols_found)

        if isempty(ik_result.sols_found{ii}')
            joint_tr(ii, :) = missing;
        else
            joint_tr(ii, :) = ik_result.sols_found{ii}';
        end

    end
    for ii=4:10
            joint_tr(:,1)=unwrap(joint_tr(:,1));
    end

    
    
    joint_tr(:,1:3)=b(ids:ide,:);
a=TForm.tform2vec(A)
%%

%% manipulability
manip = ypik.calc_manip(joint_tr(:,4:end));

%%
% ll(ids:ide)
% close all
figure('Color', [1 1 1])
subplot(1, 2, 1)
plot(tp(ids:ide,1),tp(ids:ide,2), '-.','LineWidth',2)
hold on
plot(b(ids:ide,1),b(ids:ide,2), '-.','LineWidth',2)


set(gcf,'Color',[1 1 1])
xlabel('x','fontweight','bold','fontsize',16);
ylabel('y','fontweight','bold','fontsize',16);
set(gca,'fontweight','bold','fontsize',12);

lgd =legend("Base path","Task")
lgd.FontSize = 14;
% axis equal
h=subplot(1, 2, 2)
plot(a(:,1),a(:,2), '-.','LineWidth',2)


set(gcf,'Color',[1 1 1])
xlabel('x','fontweight','bold','fontsize',16);
ylabel('y','fontweight','bold','fontsize',16);
set(gca,'fontweight','bold','fontsize',12);

lgd =legend("Task in base frame")
lgd.FontSize = 14;
% axis equal

%%


% xlim([-1 4])
close all
% figure('Color', [1 1 1])
subplot(2,1,1)

% yyaxis left
plot(ll(ids:ide),joint_tr(:, 4:end), '-.','LineWidth',2)
xlabel("Length along the task")

lgd =legend("jnt0","jnt1","jnt2","jnt3","jnt4","jnt5","jnt6")
lgd.FontSize = 14;

    set(gcf,'Color',[1 1 1])
xlabel('x','fontweight','bold','fontsize',16);
ylabel('y','fontweight','bold','fontsize',16);
set(gca,'fontweight','bold','fontsize',12);

h=subplot(2,1,2)


plot(ll(ids:ide),manip)
   
xlabel("Length along the task")

lgd =legend("Manipulability")
lgd.FontSize = 14;

    set(gcf,'Color',[1 1 1])
xlabel('x','fontweight','bold','fontsize',16);
ylabel('y','fontweight','bold','fontsize',16);
set(gca,'fontweight','bold','fontsize',12);


h.Position(2)=h.Position(2)+ h.Position(4)*0.4;
h.Position(4)=h.Position(4)*.6;

 
    
%% Cartesian validation:

% answer = ypik.get_cartesian(TForm.tform2vec(A(:,:,5)), "base_footprint", joint_tr(1, :));




%%
if false
%     sample_poses = datasample(poses, 500, 'Replace', false);
%     sample_poses = datasample(reachable_positions, 500, 'Replace', false);    
%     sample_poses = TForm.tform2vec(poses);
    sample_poses = (poses);
    Visualization.draw_poses(sample_poses, 0.05, [1 0 0], 'r');
    hold on
    Visualization.draw_poses(sample_poses, 0.05, [0 1 0], 'g');
    Visualization.draw_poses(sample_poses, 0.05, [0 0 1], 'b');
    hold off
    ros_helpers.send_rviz_poses(sample_poses, "base_footprint")
end