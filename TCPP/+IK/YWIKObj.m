classdef YWIKObj < InverseReachability.ik.IKObj

    methods

        function self = YWIKObj()
            self = self@InverseReachability.ik.IKObj();
            self.group = "youwasp_arm";
            self.ee_link = "extruder_ee";
            self.joint_names = {'vomni_joint_x', 'vomni_joint_y', 'vomni_joint_z', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5'};
            % self.base_to_arm =[0.167 0 0.142]
            self.base_to_arm = [0.167 0 0];
            self.setup_default_ik_msg()% setup the reusable obj msg
            self.setup_default_cart_msg()% setup the reusable obj msg
        end

        function get_default_seed(self)
            seed = [3; 2.14; -1.53; 2.92; 0];
        end

    end

    methods (Static)

        function constraint = get_arm_joint_2_constraint()
            % arm joint 2 cannot be bent in crap direction
            constraint = rosmessage("moveit_msgs/JointConstraint");
            constraint.JointName = "arm_joint_2";
            constraint.Position = 1.5;
            constraint.ToleranceBelow = 0.2;
            constraint.ToleranceAbove = 1.2;
            constraint.Weight = 1.1;
        end

        function constraint = get_arm_joint_1_constraint()
            constraint = rosmessage("moveit_msgs/JointConstraint");
            constraint.JointName = "arm_joint_1";
            constraint.Position = 1.5;
            constraint.ToleranceBelow = 0.2;
            constraint.ToleranceAbove = 1.2;
            constraint.Weight = 1.1;
        end

    end

end
