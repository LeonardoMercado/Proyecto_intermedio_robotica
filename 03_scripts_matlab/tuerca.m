
% Envio de puntos de pick and place a gazibo:
pose_1_izq = [0 pi/4 -pi/2 -pi/4];
%--------------------------------------------------------------------------
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
        
        % Creación del mensaje de articulaciones
        
        articulacion_1_izq = rosmessage(publicador_joint_1_izq);
        articulacion_2_izq = rosmessage(publicador_joint_2_izq);
        articulacion_3_izq = rosmessage(publicador_joint_3_izq);
        articulacion_4_izq = rosmessage(publicador_joint_4_izq);
        gripper_izq = rosmessage(publicador_gripper_izq);
        
                     r = app.EditField.Value
l = app.EditField2.Value
m = app.EditField3.Value
        
            % Puntos y orientaciones: 
            punto_orientado_1_izq = [0.0 -0.2 0.10 -90];
            punto_orientado_2_izq = [0.0 -0.2 0.06 -90];
            punto_orientado_3_izq = [0.0 -0.2 0.10 -90];
            punto_orientado_4_izq = [0.2 0.0 0.10 -90];
            punto_orientado_5_izq = [0.0 0.2 0.10 -90];
            punto_orientado_6_izq = [0.0 0.2 0.005 -90];
            punto_orientado_7_izq = [0.0 0.2 0.10 -90];
            
            %--------------------------------------------------------------------------
            
            % Pose inicial del robot en Gazibo:
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
            
            q_1_izq = solucion(punto_orientado_1_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_1_izq(1));
             app.Label10.Text = num2str(punto_orientado_1_izq(2));
             app.Label11.Text = num2str(punto_orientado_1_izq(3));
             app.Label12.Text = num2str(punto_orientado_1_izq(4));
            
            app.Label13.Text = num2str(q_1_izq(1));
            app.Label14.Text = num2str(q_1_izq(2));
            app.Label15.Text = num2str(q_1_izq(3));
            app.Label16.Text = num2str(q_1_izq(4));
            
            tg_1_izq = jtraj(pose_1_izq,q_1_izq,20);
            %robotizq.plot(tg_1_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 1');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_1_izq,1)    
                articulacion_1_izq.Data = tg_1_izq(i,1);
                articulacion_2_izq.Data = tg_1_izq(i,2);
                articulacion_3_izq.Data = tg_1_izq(i,3);
                articulacion_4_izq.Data = tg_1_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                send(publicador_gripper_izq,gripper_izq);
                pause(0.05);
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_2_izq = solucion(punto_orientado_2_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_2_izq(1));
             app.Label10.Text = num2str(punto_orientado_2_izq(2));
             app.Label11.Text = num2str(punto_orientado_2_izq(3));
             app.Label12.Text = num2str(punto_orientado_2_izq(4));
             
            app.Label13.Text = num2str(q_2_izq(1));
            app.Label14.Text = num2str(q_2_izq(2));
            app.Label15.Text = num2str(q_2_izq(3));
            app.Label16.Text = num2str(q_2_izq(4));
            
            tg_2_izq = jtraj(tg_1_izq(end,:),q_2_izq,20);
            %robotizq.plot(tg_2_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 2');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_2_izq,1)    
                articulacion_1_izq.Data = tg_2_izq(i,1);
                articulacion_2_izq.Data = tg_2_izq(i,2);
                articulacion_3_izq.Data = tg_2_izq(i,3);
                articulacion_4_izq.Data = tg_2_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
            end
            pause(1.5);
            gripper_izq.Data = [0.02,0.02];
            send(publicador_gripper_izq,gripper_izq);
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_3_izq = solucion(punto_orientado_3_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_3_izq(1));
             app.Label10.Text = num2str(punto_orientado_3_izq(2));
             app.Label11.Text = num2str(punto_orientado_3_izq(3));
             app.Label12.Text = num2str(punto_orientado_3_izq(4));
             
            app.Label13.Text = num2str(q_3_izq(1));
            app.Label14.Text = num2str(q_3_izq(2));
            app.Label15.Text = num2str(q_3_izq(3));
            app.Label16.Text = num2str(q_3_izq(4));
            
            tg_3_izq = jtraj(tg_2_izq(end,:),q_3_izq,20);
            %robotizq.plot(tg_3_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 3');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_3_izq,1)    
                articulacion_1_izq.Data = tg_3_izq(i,1);
                articulacion_2_izq.Data = tg_3_izq(i,2);
                articulacion_3_izq.Data = tg_3_izq(i,3);
                articulacion_4_izq.Data = tg_3_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_4_izq = solucion(punto_orientado_4_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_4_izq(1));
             app.Label10.Text = num2str(punto_orientado_4_izq(2));
             app.Label11.Text = num2str(punto_orientado_4_izq(3));
             app.Label12.Text = num2str(punto_orientado_4_izq(4));
             
            app.Label13.Text = num2str(q_4_izq(1));
            app.Label14.Text = num2str(q_4_izq(2));
            app.Label15.Text = num2str(q_4_izq(3));
            app.Label16.Text = num2str(q_4_izq(4));
            
            tg_4_izq = jtraj(tg_3_izq(end,:),q_4_izq,20);
            %robotizq.plot(tg_4_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 4');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_4_izq,1)    
                articulacion_1_izq.Data = tg_4_izq(i,1);
                articulacion_2_izq.Data = tg_4_izq(i,2);
                articulacion_3_izq.Data = tg_4_izq(i,3);
                articulacion_4_izq.Data = tg_4_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_5_izq = solucion(punto_orientado_5_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_5_izq(1));
             app.Label10.Text = num2str(punto_orientado_5_izq(2));
             app.Label11.Text = num2str(punto_orientado_5_izq(3));
             app.Label12.Text = num2str(punto_orientado_5_izq(4));
             
            app.Label13.Text = num2str(q_5_izq(1));
            app.Label14.Text = num2str(q_5_izq(2));
            app.Label15.Text = num2str(q_5_izq(3));
            app.Label16.Text = num2str(q_5_izq(4));
            
            tg_5_izq = jtraj(tg_4_izq(end,:),q_5_izq,20);
            %robotizq.plot(tg_5_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 5');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_5_izq,1)    
                articulacion_1_izq.Data = tg_5_izq(i,1);
                articulacion_2_izq.Data = tg_5_izq(i,2);
                articulacion_3_izq.Data = tg_5_izq(i,3);
                articulacion_4_izq.Data = tg_5_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            
            q_6_izq = solucion(punto_orientado_6_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_6_izq(1));
             app.Label10.Text = num2str(punto_orientado_6_izq(2));
             app.Label11.Text = num2str(punto_orientado_6_izq(3));
             app.Label12.Text = num2str(punto_orientado_6_izq(4));
             
            app.Label13.Text = num2str(q_6_izq(1));
            app.Label14.Text = num2str(q_6_izq(2));
            app.Label15.Text = num2str(q_6_izq(3));
            app.Label16.Text = num2str(q_6_izq(4));
            
            tg_6_izq = jtraj(tg_5_izq(end,:),q_6_izq,20);
            %robotizq.plot(tg_6_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 6');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_6_izq,1)    
                articulacion_1_izq.Data = tg_6_izq(i,1);
                articulacion_2_izq.Data = tg_6_izq(i,2);
                articulacion_3_izq.Data = tg_6_izq(i,3);
                articulacion_4_izq.Data = tg_6_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
                
            end
            pause(1.5);
            gripper_izq.Data = [0.0,0.0];
            send(publicador_gripper_izq,gripper_izq);
            
            
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_7_izq = solucion(punto_orientado_7_izq);
            %GUI
            app.Label9.Text = num2str(punto_orientado_7_izq(1));
             app.Label10.Text = num2str(punto_orientado_7_izq(2));
             app.Label11.Text = num2str(punto_orientado_7_izq(3));
             app.Label12.Text = num2str(punto_orientado_7_izq(4));
             
            app.Label13.Text = num2str(q_7_izq(1));
            app.Label14.Text = num2str(q_7_izq(2));
            app.Label15.Text = num2str(q_7_izq(3));
            app.Label16.Text = num2str(q_7_izq(4));
            
            tg_7_izq = jtraj(tg_6_izq(end,:),q_7_izq,20);
            %robotizq.plot(tg_7_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 7');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_7_izq,1)    
                articulacion_1_izq.Data = tg_7_izq(i,1);
                articulacion_2_izq.Data = tg_7_izq(i,2);
                articulacion_3_izq.Data = tg_7_izq(i,3);
                articulacion_4_izq.Data = tg_7_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);  
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            tg_8_izq = jtraj(tg_7_izq(end,:),pose_1_izq,40);
             %GUI
            app.Label9.Text = 'home';
             app.Label10.Text = 'home';
             app.Label11.Text = 'home';
             app.Label12.Text = 'home';
             
            app.Label13.Text = num2str(pose_1_izq(1));
            app.Label14.Text = num2str(pose_1_izq(2));
            app.Label15.Text = num2str(pose_1_izq(3));
            app.Label16.Text = num2str(pose_1_izq(4));
            
            %robotizq.plot(tg_8_izq,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin del punto 8')
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_8_izq,1)    
                articulacion_1_izq.Data = tg_8_izq(i,1);
                articulacion_2_izq.Data = tg_8_izq(i,2);
                articulacion_3_izq.Data = tg_8_izq(i,3);
                articulacion_4_izq.Data = tg_8_izq(i,4);
                send(publicador_joint_1_izq,articulacion_1_izq);
                send(publicador_joint_2_izq,articulacion_2_izq);
                send(publicador_joint_3_izq,articulacion_3_izq);
                send(publicador_joint_4_izq,articulacion_4_izq);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
