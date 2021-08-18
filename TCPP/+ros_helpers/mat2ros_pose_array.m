function poseArray = mat2ros_pose_array(poses,frame)
%  input: matlab matrix  each row: xyz qwqxqyqz ; frame string
%  output:  ros msg geometry_msgs/PoseArray
poseArray=rosmessage("geometry_msgs/PoseArray");
poseArray.Header.FrameId=frame; 
poseArray.Poses = arrayfun(@(~) rosmessage('geometry_msgs/Pose'),zeros(1,size(poses,1)));
for ii=1:size(poses,1)   
%     poseArray.Poses(ii)=rosmessage("geometry_msgs/Pose");
    poseArray.Poses(ii).Position.X=poses(ii,1);
    poseArray.Poses(ii).Position.Y=poses(ii,2);
    poseArray.Poses(ii).Position.Z=poses(ii,3);
    poseArray.Poses(ii).Orientation.W=poses(ii,4);
    poseArray.Poses(ii).Orientation.X=poses(ii,5);
    poseArray.Poses(ii).Orientation.Y=poses(ii,6);
    poseArray.Poses(ii).Orientation.Z=poses(ii,7);
    ii/size(poses,1)   
end

end

