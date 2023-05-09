classdef GazeboSimulator

    properties

        % gazebo positions subscriber
        sub_gazebo_positions = [];
        % joints states subscriber
        sub_joint_states = [];
        % diff drive object
        diff = struct('rate', 50)
        % arm object
        arm = struct();
        % transformations tree
        tf = []
        % reset simulation
        reset_gazebo_service = [];

       
    end

    methods

        function resetSimulation(obj)

            % RESET GAZEBO SIMULATION TO INITIAL STATE
            call(obj.reset_gazebo_service)
            pause(1)

            % set joints to 0
            obj.jointsAllControl([0 0 0 0 0 0 0]);
            obj.driveControl([0 0], 0);

            pause(1)


        end

        function obj = GazeboSimulator(rosip)

            % init ros connection
            rosshutdown
            rosinit(rosip)
            
            % simulation reset service
            obj.reset_gazebo_service = rossvcclient('/gazebo/reset_simulation');

            % tf tree watcher
            obj.tf = rostf();
            pause(1);

            % robot position subscriber
            obj.sub_gazebo_positions = rossubscriber("/gazebo/model_states");
            
            % robot arm joints states subscriber
            obj.sub_joint_states = rossubscriber("/joint_states");

            % create publisher and message for base
            obj.diff.pub = rospublisher('/mobile_base_controller/cmd_vel');
            obj.diff.msg = rosmessage(obj.diff.pub);
            
            % create publishers for arm
            obj.arm.pub(1) = rospublisher("/controller_joint_1_position/command");
            obj.arm.pub(2) = rospublisher("/controller_joint_2_position/command");
            obj.arm.pub(3) = rospublisher("/controller_joint_3_position/command");
            obj.arm.pub(4) = rospublisher("/controller_joint_4_position/command");
            obj.arm.pub(5) = rospublisher("/controller_joint_5_position/command");
            obj.arm.pub(6) = rospublisher("/controller_joint_6_position/command");
            obj.arm.pub(7) = rospublisher("/controller_joint_7_position/command");
                       
            % create message objects for arm            
            obj.arm.msg(1) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(2) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(3) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(4) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(5) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(6) = rosmessage(obj.arm.pub(1));
            obj.arm.msg(7) = rosmessage(obj.arm.pub(1));

        
        end

        function jointControl(obj, jointNum, angle)

            % CONTROL JOINT - control one joint 
            %
            % ----------------------------------
            % in:
            %
            % jointNum : 1 - 7
            % angle: jointAngle in rad
            %
            % ----------------------------------

            % save joint value to arm object
            obj.arm.msg(jointNum).Data = angle;

            % send move command
            obj.arm.pub(jointNum).send(obj.arm.msg(jointNum));

        end

        function jointsAllControl(obj, angles)

            % CONTROL JOINT - control all joints 
            %
            % ----------------------------------
            % in:
            %
            % jointNum : 1 - 7
            % angle: jointAngle in rad
            %
            % ----------------------------------

            obj.jointControl(1, angles(1))
            obj.jointControl(2, angles(2))
            obj.jointControl(3, angles(3))
            obj.jointControl(4, angles(4))
            obj.jointControl(5, angles(5))
            obj.jointControl(6, angles(6))
            obj.jointControl(7, angles(7))            


        end

        function angle = jointRead(obj, jointNum)

            % READ JOINT - get current position of the joint
            %
            % ----------------------------------
            %
            % in: jointNum : 1-7
            %
            % out: angle - joint angle
            %
            % ----------------------------------

            data = obj.sub_joint_states.receive();
            angle = data.Position(jointNum)

        end

        function angles = jointsAllRead(obj)

            % READ JOINT - get current position of arm joints
            %
            % ----------------------------------
            %
            % out: angles - joints angles
            %
            % ----------------------------------

            data = obj.sub_joint_states.receive();
            angles = data.Position(1:7)

        end

        function driveControl(obj, velocity, duration)

            % CONTROL DIFF DRIVE
            % 
            % ----------------------------------
            %
            % in: 
            % 
            % velocity : [v, w]
            %
            % duration : time in sec ( 0 - inf )
            %   
            % ----------------------------------

            obj.diff.msg.Linear.X =  velocity(1);
            obj.diff.msg.Angular.Z = velocity(2);

            t = tic();
            r = rosrate(obj.diff.rate);
            
            % run control loop for chosen duration
            while 1
                
                % send velocity
                obj.diff.pub.send(obj.diff.msg)


                % stop loop after duration
                if (duration < toc(t))
                    break;
                end

                % set execution frequency
                waitfor(r)
   
            end

        end

        function T = getPositionEE(obj)
        
            % GET EE POSITION
            %
            % ----------------------------------
            %
            % output: T ( 4x4 transf matrix : world <-> EE ) 
            %
            % ----------------------------------

%             waitForTransform(obj.tf, 'panda_link7', 'world')
            p7world = getTransform(obj.tf, 'world', 'panda_link7');
            RR = quat2rotm(p7world.Transform.Rotation.readQuaternion);
            p = [p7world.Transform.Translation.X p7world.Transform.Translation.Y  p7world.Transform.Translation.Z ];
            T = eye(4);
            T(1:3,1:3) = RR;
            T(1:3,4) = p';

        end

        function T = getTransformation(obj, frame1, frame2)

            % GET JOINT POSITIONS
            %
            % ----------------------------------
            %
            % output: T ( 4x4 transf matrixs : world <-> EE ) 
            %
            % ----------------------------------

            transf = getTransform(obj.tf, frame1, frame2);
            rotm = quat2rotm(transf.Transform.Rotation.readQuaternion);
%             transf.Transform.Translation

            T = eye(4);
            T(1:3,1:3) = rotm;
            T(1,4) = transf.Transform.Translation.X;
            T(2,4) = transf.Transform.Translation.Y;
            T(3,4) = transf.Transform.Translation.Z;
            


        end

        function fi_base = getBaseAngle(obj)
        
            % GET BASE ANGLE - get base angle in the world coordinates
            %
            % ----------------------------------
            %
            % output: fi_base ( angle in rad ) 
            %
            % ----------------------------------
    
            % TF BASE -> WORLD
            baseworld = getTransform(obj.tf, 'world', 'base_link');
            
            % get angle of base platform rotation
            base_rotat = quat2axang(quaternion([baseworld.Transform.Rotation.W baseworld.Transform.Rotation.X baseworld.Transform.Rotation.Y baseworld.Transform.Rotation.Z]));
            fi_base = sign(base_rotat(3))*base_rotat(4);      

        end

        function publishRobotPosition(obj)

            % PUBLISH ROBOT POSITION FROM GAZEBO TO TF
            %
            % receives position of robot in world TF from gazebo 
            % and publishes it to TF tree
            %
            % ----------------------------------
                    
            msg = obj.sub_gazebo_positions.receive();
            position = msg.Pose(2).Position;
            orientation = msg.Pose(2).Orientation;
            
            
            tfStampedMsg = rosmessage('geometry_msgs/TransformStamped');
            tfStampedMsg.ChildFrameId = 'base_footprint';
            tfStampedMsg.Header.FrameId = 'world';
            
            tfStampedMsg.Transform.Translation.X = position.X;
            tfStampedMsg.Transform.Translation.Y = position.Y;
            tfStampedMsg.Transform.Translation.Z = position.Z;
            
            tfStampedMsg.Transform.Rotation.W = orientation.W;
            tfStampedMsg.Transform.Rotation.X = orientation.X;
            tfStampedMsg.Transform.Rotation.Y = orientation.Y;
            tfStampedMsg.Transform.Rotation.Z = orientation.Z;
        
            tfStampedMsg.Header.Stamp = rostime('now');
            
            sendTransform(obj.tf, tfStampedMsg)
        
        
        end



    end
end