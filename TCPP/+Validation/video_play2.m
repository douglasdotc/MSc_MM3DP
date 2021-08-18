% 
% load("PVid1___1111New");
% 
% %% Video:
% joint_tr=video_struct.joint_tr;
% T=video_struct.T;
% tp=video_struct.tp;
% a=video_struct.a;
% b=video_struct.b;
% hz=video_struct.hz;
% % n=size(tp,1)-mod(size(tp,1),hz);
% n=size(tp,1);
% m=size(T,3);
% 
% ids=1:hz:n;
% nnn=length(ids);
% %% deal with solution
% points=(arrayfun(@(~) rosmessage('geometry_msgs/Point'), zeros(1, length(ids))));
% % points=(arrayfun(@(~) rosmessage('geometry_msgs/Point'), zeros(1, size(T,3))));
% % points=task2points(T,points);
% points=task2points(tp,points,hz);
% %%  ROS
% % rosshutdown
% % rosinit
% pasttaskpub=rospublisher("past_task","visualization_msgs/Marker");
% taskpub=rospublisher("task","visualization_msgs/Marker");
% taskpub2=rospublisher("task2","visualization_msgs/Marker");
% jointpub=rospublisher("video_joint_states","sensor_msgs/JointState");
% %%
% speedup=4;
% %% Environment:
% F=[[-.5,-.5,2-(.25/2),.25];
% [1.5-(.25/2),-.5,0.25,2];
% [1.5-(.25/2),2.5,0.25,.75];
% [1.5-(.25/2),3.25,3.25-1.5+(.25/2)+.15,.25];
% [3.25-.2+.15,3.25-3.75,.2,3.75]];
% P=[1.5,-0.7,1.5,.45];
% load_environment(P)
% % load_environment(F)
% 
% %% Default Messages
% task_msg=get_default_task_msg();
% joint_msg=get_default_joint_msg();
%%

% id=1;
%id=round(3*n/13);

%% plot till id
% id_=1;
% while id_ <=id        
%     task_msg.Id = id_+50;
%     
%     if mod((id_-1),hz)==0
%         
%         
%         iid=((id_-1)/hz)+1;
%         task_msg.Points= points(iid:iid+1);   
%         taskpub.send(task_msg);
%         
%         
%     end
%         
%     id_=id_+1;
%   
% end
% t0=tic;
% t_last=toc(t0);
% while id <=n-1        
%     task_msg.Id = id+50;    
%     
%     if mod(id,hz)==2
%         iid=ceil((id/n)*nnn);
%         task_msg.Points= points(iid:iid+1);    
%         %task_msg.Points= points(iid:iid+1);    
%         
%         
%          if points(iid).Z>0.01
%              
%             task_msg.Color.R=0.1961;
%             task_msg.Color.G=0.4824;
%             task_msg.Color.B=0.9412;
%             task_msg.Color.A=1;
%          
%          end
%         taskpub.send(task_msg);
%         
%         
%     end
%     
%     %if mod((id-1),hz)==0
%     %    iid=((id-1)/hz)+1;
%     %    task_msg.Points= points(iid+3:iid+4);    
%         %task_msg.Points= points(iid:iid+1);    
%     %    taskpub.send(task_msg);  
%     %end
%     
%     
%     
%     if ~any(  isnan(joint_tr(id,:))   )
%     joint_msg.Position=joint_tr(id,:);    
%     jointpub.send(joint_msg);
%     end
% 
%         while (toc(t0)-t_last)<(1/(hz*speedup))
%         %     
%         end
% 
%     t_last=toc(t0);
%     id=id+1;
%     disp(id)
% end
%%
if true
    pause(3)
        for ii=1:size(full_reposition,1)  

          
            joint_msg.Position=full_reposition(ii,:);    
            jointpub.send(joint_msg);       
            pause(0.05);
            
        end
            
end

%% %%%%%%%%%%%%%%%%%%
% 
% load("PVid22__22New");
% 
% %%
% %% Video:
% joint_tr=video_struct.joint_tr;
% T=video_struct.T;
% tp=video_struct.tp;
% a=video_struct.a;
% b=video_struct.b;
% hz=video_struct.hz;
% % n=size(tp,1)-mod(size(tp,1),hz);
% n=size(tp,1);
% m=size(T,3);
% ids=1:hz:n;
% nnn=length(ids);
% %% deal with solution
% points=(arrayfun(@(~) rosmessage('geometry_msgs/Point'), zeros(1, length(ids))));
% 
% points=task2points(tp,points,hz);
% %%

%% plot till id
pause(3)
id=1
id_=id;
t0=tic;
t_last=toc(t0);
while id <=n-1        
    task_msg.Id = id+50+id_;    
    
    if mod(id,hz)==2
        iid=ceil((id/n)*nnn);
        task_msg.Points= points(iid:iid+1);    
        %task_msg.Points= points(iid:iid+1);    
        
        
         if points(iid).Z>0.01
             
            task_msg.Color.R=0.1961;
            task_msg.Color.G=0.4824;
            task_msg.Color.B=0.9412;
            task_msg.Color.A=1;
         
         end
        taskpub.send(task_msg);
        
        
    end
    
    %if mod((id-1),hz)==0
    %    iid=((id-1)/hz)+1;
    %    task_msg.Points= points(iid+3:iid+4);    
        %task_msg.Points= points(iid:iid+1);    
    %    taskpub.send(task_msg);  
    %end
    
    
    
    if ~any(  isnan(joint_tr(id,:))   )
    joint_msg.Position=joint_tr(id,:);    
    jointpub.send(joint_msg);
    end

        while (toc(t0)-t_last)<(1/(hz*speedup))
        %     
        end

    t_last=toc(t0);
    id=id+1;
    disp(id)
end



%% %%%%%%%%%%%%%%%%%
function points=task2points(tp,points,hz)
%     n=size(T,3);
    mm=size(tp,1);
ids=1:hz:mm;
    for ii=1:length(ids)
        
      
       
           points(ii).X=tp(ids(ii),1);
           points(ii).Y=tp(ids(ii),2);
           points(ii).Z=tp(ids(ii),3);

        
        
          
%          
%             points(ii).X=T(1,4,ii);
%             points(ii).Y=T(2,4,ii);
%             points(ii).Z=T(3,4,ii);
    
            
        
    end

end


function load_environment(obstacles)
obspub=rospublisher("obstacles","visualization_msgs/Marker");

env_msg = rosmessage("visualization_msgs/Marker");
env_msg.Action = env_msg.MODIFY;
env_msg.Type = env_msg.CUBE;
env_msg.FrameLocked = false;
env_msg.Header.FrameId = "world";

c = [0.4940 0.1840 0.4560]*0.3;

env_msg.Color.R=c(1);
env_msg.Color.G=c(2);
env_msg.Color.B=c(3);
env_msg.Color.A=1;
  
    for ii=1:size(obstacles,1)
    env_msg.Id = ii;
    env_msg.Scale.X = obstacles(ii,3);
    env_msg.Scale.Y =obstacles(ii,4);
    env_msg.Scale.Z = .5;
    env_msg.Pose.Position.X = obstacles(ii,1)+(obstacles(ii,3)/2);
    env_msg.Pose.Position.Y = obstacles(ii,2)+(obstacles(ii,4)/2);
    env_msg.Pose.Position.Z = 0.25;
    env_msg.Pose.Orientation.W = 1;
    obspub.send(env_msg)
    pause(1)
    end
%pause(1)

end

function task_msg=get_default_task_msg()
task_msg = rosmessage("visualization_msgs/Marker");
task_msg.Action = task_msg.MODIFY;
task_msg.Type = task_msg.LINELIST;
task_msg.FrameLocked = false;
task_msg.Id = 1;
task_msg.Header.FrameId = "world";
task_msg.Scale.X = 0.01;
task_msg.Scale.Y = 0.01;
task_msg.Scale.Z = 0.01;
task_msg.Pose.Orientation.W = 1;

task_msg.Color.R=0.7098;
task_msg.Color.G=0.2235;
task_msg.Color.B=0.1294;
task_msg.Color.A=1;
end

function joint_msg=get_default_joint_msg()
joint_names={'vomni_joint_x', 'vomni_joint_y', 'vomni_joint_z', 'panda_joint1', 'panda_joint2', 'panda_joint4', 'panda_joint6'};
% joint_names={'vomni_joint_x', 'vomni_joint_y', 'vomni_joint_z', 'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'};
joint_msg = rosmessage("sensor_msgs/JointState");
joint_msg.Name=joint_names;
joint_msg.Position=zeros(1,7);
end