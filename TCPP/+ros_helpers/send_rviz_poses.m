function [pub, msg]=send_rviz_poses(poses,link,varargin)
% Input: an array of poses objects
% Output:   sends posearray msg to ros network
topic="matlab/pose_array";
if ~isempty(varargin)
    topic=varargin{1};
end
pub=rospublisher(topic,"geometry_msgs/PoseArray");
pause(0.5);% This is needed for some weird reason
msg=ros_helpers.mat2ros_pose_array(poses,link);
pub.send(msg);
end

