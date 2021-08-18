function poseStamped = vec2ros_pose_stamped(pose,frame)
%  input: matlab vector xyz qwqxqyqz ; frame string
%  output:  ros msg geometry_msgs/PoseStamped
poseStamped=rosmessage("geometry_msgs/PoseStamped");
poseStamped.Header.FrameId=frame;
poseStamped.Pose.Position.X=pose(1);
poseStamped.Pose.Position.Y=pose(2);
poseStamped.Pose.Position.Z=pose(3);
poseStamped.Pose.Orientation.W=pose(4);% Note  matlab represents quaterions as WXYZ
poseStamped.Pose.Orientation.X=pose(5);
poseStamped.Pose.Orientation.Y=pose(6);
poseStamped.Pose.Orientation.Z=pose(7);
end

