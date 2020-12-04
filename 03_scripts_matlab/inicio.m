       
        pose_1_izq = [0 pi/4 -pi/2 -pi/4];
         
        
        pose_1 = [0 pi/4 -pi/2 -pi/4];
      
        % Gazebo + matlab + ROS:
        
        % Publicadores a los controladores de las articulaciones:
       publicador_joint_1 = rospublisher('/robot_derecha/joint1_position_controller/command','std_msgs/Float64');
        pause(1);
         publicador_joint_2 = rospublisher('/robot_derecha/joint2_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_joint_3 = rospublisher('/robot_derecha/joint3_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_joint_4 = rospublisher('/robot_derecha/joint4_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_gripper  = rospublisher('/robot_derecha/gripper_position_controller/command','std_msgs/Float64MultiArray');
        pause(1);
        disp('publicadores creados')
       
        
        % Creación del mensaje de articulaciones
        
        articulacion_1 = rosmessage(publicador_joint_1);
        articulacion_2 = rosmessage(publicador_joint_2);
        articulacion_3 = rosmessage(publicador_joint_3);
        articulacion_4 = rosmessage(publicador_joint_4);
        gripper = rosmessage(publicador_gripper);
        
        art1 = articulacion_1;
        % Configuración y envio de datos de las articulaciones:
        
        articulacion_1.Data = pose_1(1);
        articulacion_2.Data = pose_1(2);
        articulacion_3.Data = pose_1(3);
        articulacion_4.Data = pose_1(4);
        gripper.Data = [0.0,0.0];
        send(publicador_joint_1,articulacion_1);
        send(publicador_joint_2,articulacion_2);
        send(publicador_joint_3,articulacion_3);
        send(publicador_joint_4,articulacion_4);
        send(publicador_gripper,gripper);
        pause(1);
        
        %Gazebo + matlab + ROS:
        
        % Publicadores a los controladores de las articulaciones:
        
        publicador_joint_1_izq = rospublisher('/robot_izquierda/joint1_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_joint_2_izq = rospublisher('/robot_izquierda/joint2_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_joint_3_izq = rospublisher('/robot_izquierda/joint3_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_joint_4_izq = rospublisher('/robot_izquierda/joint4_position_controller/command','std_msgs/Float64');
        pause(1);
        publicador_gripper_izq = rospublisher('/robot_izquierda/gripper_position_controller/command','std_msgs/Float64MultiArray');
        pause(1);
        disp('publicadores creados izq')
        
        app.Lamp5.Color = 'g';
        % Creación del mensaje de articulaciones
        
        articulacion_1_izq = rosmessage(publicador_joint_1_izq);
        articulacion_2_izq = rosmessage(publicador_joint_2_izq);
        articulacion_3_izq = rosmessage(publicador_joint_3_izq);
        articulacion_4_izq = rosmessage(publicador_joint_4_izq);
        gripper_izq = rosmessage(publicador_gripper_izq);
        
        % Configuración y envio de datos de las articulaciones:
        
        articulacion_1_izq.Data = pose_1_izq(1);
        articulacion_2_izq.Data = pose_1_izq(2);
        articulacion_3_izq.Data = pose_1_izq(3);
        articulacion_4_izq.Data = pose_1_izq(4);
        gripper_izq.Data = [0.0,0.0];
        send(publicador_joint_1_izq,articulacion_1_izq);
        send(publicador_joint_2_izq,articulacion_2_izq);
        send(publicador_joint_3_izq,articulacion_3_izq);
        send(publicador_joint_4_izq,articulacion_4_izq);
        send(publicador_gripper_izq,gripper_izq);
        
    
        %subcriptor_izq = rossubscriber('/robot_izquierda/joint_states');
        pause(1);
        
        %%GUI
        
        app.Label5.Text = num2str(pose_1(1));
        app.Label6.Text = num2str(pose_1(2));
        app.Label7.Text = num2str(pose_1(3));
        app.Label8.Text = num2str(pose_1(4));
        
        app.Label13.Text = num2str(pose_1_izq(1));
        app.Label14.Text = num2str(pose_1_izq(2));
        app.Label15.Text = num2str(pose_1_izq(3));
        app.Label16.Text = num2str(pose_1_izq(4));
        
            
        app.Label.Text = 'home';
        app.Label2.Text = 'home';
        app.Label3.Text = 'home';
        app.Label4.Text = 'home';
        
        app.Label9.Text = 'home';
        app.Label10.Text = 'home';
        app.Label11.Text = 'home';
        app.Label12.Text = 'home';
        
    
