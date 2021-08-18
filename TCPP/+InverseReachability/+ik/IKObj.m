classdef IKObj < handle

    properties
        ik_request_msg
        cart_request_msg
        ik_client
        cart_client
        jac_client
        base_to_arm
        constraints = containers.Map()
        group = "ik_group"
        ee_link = "ee_link"
        joint_names = {"jnt1, jnt2"}
    end

    methods

        function self = IKObj()
            self.check_ros_setup()%Check that ros network is up
        end

        function ik = get_ik(self, target_vec, frame, varargin)
            %  set seed, constant, dynamic

            seed = self.get_default_seed()';
            base_jnts = zeros(1, 3);
            use_last = false;
            self.ik_request_msg.IkRequest.Attempts = 3;
            self.ik_request_msg.IkRequest.Timeout = rosduration(.01);

            while ~isempty(varargin)
                assert(mod(length(varargin), 2) == 0);

                switch lower(varargin{1})
                    case "seed"
                        seed = varargin{2};
                    case "attempts"
                        self.ik_request_msg.IkRequest.Attempts = varargin{2};
                    case "timeout"
                        self.ik_request_msg.IkRequest.Timeout = rosduration(varargin{2});
                    case "use_last"
                        use_last = varargin{2};
                    case "base_jnts"
                        base_jnts = varargin{2};
                    otherwise
                        error(['Unexpected option: ' varargin{1}])
                end

                varargin(1:2) = [];
            end

            if size(base_jnts, 1) == 1
                self.ik_request_msg.IkRequest.RobotState.JointState.Position(1:3) = base_jnts;
            end

            if size(seed, 1) == 1
                self.ik_request_msg.IkRequest.RobotState.JointState.Position(4:end) = seed;
            else
                assert(~use_last); % if seed vec is provided,  previous ik should not be used.
            end

            %Offset target_vec to be centered round the arm
            answers = ones(size(target_vec, 1), 1) * -999;
            sols = cell(size(target_vec, 1), 1);
            tic

            for ii = 1:size(target_vec, 1)

                self.ik_request_msg.IkRequest.PoseStamped = ros_helpers.vec2ros_pose_stamped(target_vec(ii, :), frame); %"base_link";%Remember the transform to arm_link_1

                
                self.ik_request_msg.IkRequest.RobotState.JointState.Position(1:3) = base_jnts(min(size(base_jnts, 1),ii), :);
                self.ik_request_msg.IkRequest.RobotState.JointState.Position(4:end) = seed(min(size(seed, 1),ii), :);
                

                %Request
                answer = self.ik_client.call(self.ik_request_msg);
                %Save answer validity and joint positions
                answers(ii) = answer.ErrorCode.Val;
                sols{ii} = answer.Solution.JointState.Position;

                % set seed:
                if use_last && answer.ErrorCode.Val == 1
                    seed = answer.Solution.JointState.Position(4:end)';
                end

                % Time left info
                t = toc;
                [h, m, s] = hms(seconds(-t + t / (ii / size(target_vec, 1))));
                disp([ii / size(target_vec, 1), h, m, s])
            end

            ik.tested_vec = target_vec;
            ik.received_answers = answers;
            ik.sols_found = sols;
            ik.ik_exists_vec = target_vec(answers == 1, :);
        end

        function solution = validate_path(self, task_vec, frame)

            % %Offset target_vec to be centered round the arm
            % task_vec = task_vec + repmat([self.base_to_arm zeros(1, 4)], size(task_vec, 1), 1);

            % self.cart_request_msg.StartState.JointState.Position = zeros(1, length(self.joint_names));
            % longest_frac = 0;
            % longest_solution = [];
            % tic

            % while true
            %     %%

            %     %%
            %     answer = self.cart_client.call(self.cart_request_msg);
            %     answer.Fraction
            %     answer.Solution
            %     answer.StartState

            %     if answer.Fraction.Data > max_frac
            %         max_frac = answer.Fraction.Data;
            %         longest_solution = answer.Solution;
            %     end

            %     % Time left info
            %     t = toc;
            %     [h, m, s] = hms(seconds(-t + t / (ii / size(task_vec, 1))));
            %     disp([ii / size(task_vec, 1), h, m, s])
            % end

        end

        function answers = genManipulability(arm_joints)

            try
                rosinit
            catch
                disp("MATLAB ROS node already initialised")
            end

            try
                jac_client = rossvcclient("kdl_kinematics_services/getJac", "timeout", 5);

            catch
                disp("getJac service not found")
                return
            end

            %% Request loop
            %% Request loop
            request = jac_client.rosmessage;
            answers = zeros(size(arm_joints, 1), 1);
            tic

            for ii = 1:size(arm_joints, 1)
                request.JointState.Position = arm_joints(ii, :);
                answer = jac_client.call(request);
                jacm = reshape(vertcat(answer.Jacobian(:).Data), 5, 6)';
                jacm = jacm(1:3, :); % pos only?
                answers(ii) = det(jacm * (jacm'));
                % Time left info
                t = toc;
                [h, m, s] = hms(seconds(-t + t / (ii / size(arm_joints, 1))));
                disp([ii / size(arm_joints, 1), h, m, s])
            end

            rosshutdown;
        end

        % end

        % methods (Access = private)

        function check_ros_setup(self)

            try
                rosinit
            catch
                disp("MATLAB ROS node already initialised")
            end

            if isempty(self.ik_client)

                try
                    self.ik_client = rossvcclient("compute_ik", "timeout", 5);
                catch
                    disp("compute_ik service not found")
                    return
                end

            end

            if isempty(self.cart_client)

                try
                    self.cart_client = rossvcclient("compute_cartesian_path", "timeout", 5);
                catch
                    disp("compute_cartesian_path service not found")
                    return
                end

            end

        end

        function setup_default_ik_msg(self)
            % Setup the reusable obj message.
            self.ik_request_msg = self.ik_client.rosmessage;
            self.ik_request_msg.IkRequest.GroupName = self.group;
            self.ik_request_msg.IkRequest.AvoidCollisions = true;
            self.ik_request_msg.IkRequest.IkLinkName = self.ee_link;
            self.ik_request_msg.IkRequest.Timeout = rosduration(.005);
            state = robotics.ros.msggen.moveit_msgs.RobotState;
            state.JointState.Name = self.joint_names;
            state.JointState.Position = zeros(1, length(self.joint_names));
            self.ik_request_msg.IkRequest.RobotState = state;
            self.ik_request_msg.IkRequest.Attempts = 5;
        end

        function setup_default_cart_msg(self)

            % Setup the reusable obj message.
            self.cart_request_msg = self.cart_client.rosmessage;

            state = robotics.ros.msggen.moveit_msgs.RobotState;
            state.JointState.Name = self.joint_names;
            state.JointState.Position = zeros(1, length(self.joint_names));
            self.cart_request_msg.StartState = state; %TODO

            self.cart_request_msg.GroupName = self.group;
            self.cart_request_msg.LinkName = self.ee_link;

            self.cart_request_msg.MaxStep = 0.01;
            self.cart_request_msg.JumpThreshold = 0.05;

            self.cart_request_msg.AvoidCollisions = true;

        end

        function add_constraints(self, constraint)
            % append constraint msg to request msg
            if isempty(self.ik_request_msg.Constraints.Name)
                self.ik_request_msg.Constraints.Name = "Constraints";
            end

            switch class(constraint)
                case 'ros.msggen.moveit_msgs.JointConstraint'
                    self.ik_request_msg.Constraints.JointConstraints = [self.ik_request_msg.Constraints.JointConstraints constraint];
                case 'ros.msggen.moveit_msgs.PositionConstraint'
                    self.ik_request_msg.Constraints.PositionConstraint = [self.ik_request_msg.Constraints.PositionConstraint constraint];
                case 'ros.msggen.moveit_msgs.OrientationConstraint'
                    self.ik_request_msg.Constraints.OrientationConstraint = [self.ik_request_msg.Constraints.OrientationConstraint constraint];
                otherwise
                    disp "Constraint not supported"
            end

        end

    end

end
