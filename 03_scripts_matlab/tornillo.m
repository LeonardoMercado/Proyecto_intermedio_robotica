
           
        pose_1 = [0 pi/4 -pi/2 -pi/4];
             % Puntos y orientaciones: 
             r = app.EditField.Value
l = app.EditField2.Value
m = app.EditField3.Value
            %punto_orientado_1 = [0.0 0.2 0.10 -90];
            punto_orientado_1 = [r l m -90];
            punto_orientado_2 = [0.0 0.2 0.03 -90];
            punto_orientado_3 = [0.0 0.2 0.10 -90];
            punto_orientado_4 = [0.2 0.0 0.10 -90];
            punto_orientado_5 = [0.0 -0.2 0.10 -90];
            punto_orientado_6 = [0.0 -0.2 0.07 -90];
            punto_orientado_7 = [0.0 -0.2 0.10 -90];
            
            %--------------------------------------------------------------------------
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
        
        
            % Pose inicial del robot en Gazibo:
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
            
            
            q_1 = solucion(punto_orientado_1);
            tg_1 = jtraj(pose_1,q_1,20);
            %GUI
             app.Label.Text = num2str(punto_orientado_1(1));
             app.Label2.Text = num2str(punto_orientado_1(2));
             app.Label3.Text = num2str(punto_orientado_1(3));
             app.Label4.Text = num2str(punto_orientado_1(4));
            
             app.Label5.Text = num2str(q_1(1));
             app.Label6.Text = num2str(q_1(2));
             app.Label7.Text = num2str(q_1(3));
             app.Label8.Text = num2str(q_1(4));
            %robotder.plot(tg_1,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 1');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_1,1)    
                articulacion_1.Data = tg_1(i,1);
                articulacion_2.Data = tg_1(i,2);
                articulacion_3.Data = tg_1(i,3);
                articulacion_4.Data = tg_1(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                send(publicador_gripper,gripper);
                pause(0.05);
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_2 = solucion(punto_orientado_2);
            tg_2 = jtraj(tg_1(end,:),q_2,20);
            %GUI
             app.Label.Text = num2str(punto_orientado_2(1));
             app.Label2.Text = num2str(punto_orientado_2(2));
             app.Label3.Text = num2str(punto_orientado_2(3));
             app.Label4.Text = num2str(punto_orientado_2(4));

           app.Label5.Text = num2str(q_2(1));
             app.Label6.Text = num2str(q_2(2));
             app.Label7.Text = num2str(q_2(3));
             app.Label8.Text = num2str(q_2(4));
             
         
            
            %robotder.plot(tg_2,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 2');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_2,1)    
                articulacion_1.Data = tg_2(i,1);
                articulacion_2.Data = tg_2(i,2);
                articulacion_3.Data = tg_2(i,3);
                articulacion_4.Data = tg_2(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
            end
            pause(1.5);
            gripper.Data = [0.02,0.02];
            send(publicador_gripper,gripper);
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_3 = solucion(punto_orientado_3);
              %GUI
              
             app.Label.Text = num2str(punto_orientado_3(1));
             app.Label2.Text = num2str(punto_orientado_3(2));
             app.Label3.Text = num2str(punto_orientado_3(3));
             app.Label4.Text = num2str(punto_orientado_3(4));
              
             app.Label5.Text = num2str(q_3(1));
             app.Label6.Text = num2str(q_3(2));
             app.Label7.Text = num2str(q_3(3));
             app.Label8.Text = num2str(q_3(4));
            tg_3 = jtraj(tg_2(end,:),q_3,20);
            %robotder.plot(tg_3,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 3');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_3,1)    
                articulacion_1.Data = tg_3(i,1);
                articulacion_2.Data = tg_3(i,2);
                articulacion_3.Data = tg_3(i,3);
                articulacion_4.Data = tg_3(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_4 = solucion(punto_orientado_4);
              %GUI
              app.Label.Text = num2str(punto_orientado_4(1));
             app.Label2.Text = num2str(punto_orientado_4(2));
             app.Label3.Text = num2str(punto_orientado_4(3));
             app.Label4.Text = num2str(punto_orientado_4(4));
             
           app.Label5.Text = num2str(q_4(1));
             app.Label6.Text = num2str(q_4(2));
             app.Label7.Text = num2str(q_4(3));
             app.Label8.Text = num2str(q_4(4));
             
            tg_4 = jtraj(tg_3(end,:),q_4,20);
            %robotder.plot(tg_4,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 4');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_4,1)    
                articulacion_1.Data = tg_4(i,1);
                articulacion_2.Data = tg_4(i,2);
                articulacion_3.Data = tg_4(i,3);
                articulacion_4.Data = tg_4(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_5 = solucion(punto_orientado_5);
            tg_5 = jtraj(tg_4(end,:),q_5,20);
              %GUI
              app.Label.Text = num2str(punto_orientado_5(1));
             app.Label2.Text = num2str(punto_orientado_5(2));
             app.Label3.Text = num2str(punto_orientado_5(3));
             app.Label4.Text = num2str(punto_orientado_5(4));
              
             app.Label5.Text = num2str(q_5(1));
             app.Label6.Text = num2str(q_5(2));
             app.Label7.Text = num2str(q_5(3));
             app.Label8.Text = num2str(q_5(4));
            
            %robotder.plot(tg_5,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 5');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_5,1)    
                articulacion_1.Data = tg_5(i,1);
                articulacion_2.Data = tg_5(i,2);
                articulacion_3.Data = tg_5(i,3);
                articulacion_4.Data = tg_5(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            
            q_6 = solucion(punto_orientado_6);
            tg_6 = jtraj(tg_5(end,:),q_6,20);
            
              %GUI
              app.Label.Text = num2str(punto_orientado_6(1));
             app.Label2.Text = num2str(punto_orientado_6(2));
             app.Label3.Text = num2str(punto_orientado_6(3));
             app.Label4.Text = num2str(punto_orientado_6(4));
             
             app.Label5.Text = num2str(q_6(1));
             app.Label6.Text = num2str(q_6(2));
             app.Label7.Text = num2str(q_6(3));
             app.Label8.Text = num2str(q_6(4));
            
            %robotder.plot(tg_6,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 6');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_6,1)    
                articulacion_1.Data = tg_6(i,1);
                articulacion_2.Data = tg_6(i,2);
                articulacion_3.Data = tg_6(i,3);
                articulacion_4.Data = tg_6(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
                
            end
            pause(1.5);
            gripper.Data = [0.0,0.0];
            send(publicador_gripper,gripper);
            
            
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            q_7 = solucion(punto_orientado_7);
            tg_7 = jtraj(tg_6(end,:),q_7,20);
            
              %GUI
              app.Label.Text = num2str(punto_orientado_7(1));
             app.Label2.Text = num2str(punto_orientado_7(2));
             app.Label3.Text = num2str(punto_orientado_7(3));
             app.Label4.Text = num2str(punto_orientado_7(4));
              
             app.Label5.Text = num2str(q_7(1));
             app.Label6.Text = num2str(q_7(2));
             app.Label7.Text = num2str(q_7(3));
             app.Label8.Text = num2str(q_7(4));
            
            %robotder.plot(tg_7,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin punto 7');
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_7,1)    
                articulacion_1.Data = tg_7(i,1);
                articulacion_2.Data = tg_7(i,2);
                articulacion_3.Data = tg_7(i,3);
                articulacion_4.Data = tg_7(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);  
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
            
            
            tg_8 = jtraj(tg_7(end,:),pose_1,40);
              %GUI
              app.Label.Text = 'home';
             app.Label2.Text = 'home';
             app.Label3.Text = 'home';
             app.Label4.Text = 'home';
              
             app.Label5.Text = num2str(pose_1(1));
             app.Label6.Text = num2str(pose_1(2));
             app.Label7.Text = num2str(pose_1(3));
             app.Label8.Text = num2str(pose_1(4));
             
            %robotder.plot(tg_8,'workspace', maximo,'noa','view',[30 30]);
            disp('Fin del punto 8')
            disp('----------------------------ENVIANDO A GAZIBO--------------------');
            for i=1:size(tg_8,1)    
                articulacion_1.Data = tg_8(i,1);
                articulacion_2.Data = tg_8(i,2);
                articulacion_3.Data = tg_8(i,3);
                articulacion_4.Data = tg_8(i,4);
                send(publicador_joint_1,articulacion_1);
                send(publicador_joint_2,articulacion_2);
                send(publicador_joint_3,articulacion_3);
                send(publicador_joint_4,articulacion_4);
                pause(0.05);
                
            end
            disp('----------------------------FIN ENVIANDO A GAZIBO--------------------');
