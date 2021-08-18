function viz_poses(poses)
% send the poses to  rviz for visualisation
pub=rospublisher("matlab_poses","geometry_msgs/PoseArray");
pub.send(poses);
% Can I delete the pub ?
end

