%% Script para el desarrollo del proyecto intermedio de robótica 2020-II
%
%
%
%%  Montaje del robot y validación del modelo:
clc;
clear;
close all; 

syms q1 q2 q3 q4

l1 = 0.135875;
l2 = 0.107;
l3 = 0.107;
l4 = 0.091;
%l4 = 0.108;


L(1) = Link('revolute','alpha', 0,    'a',0,   'd',l1,  'offset', 0,   'modified', 'qlim',[-2*pi 2*pi]);
L(2) = Link('revolute','alpha', pi/2, 'a',0,   'd',0,   'offset', pi/2, 'modified', 'qlim',[-2*pi 2*pi]);
L(3) = Link('revolute','alpha', 0,    'a',l2,  'd',0,   'offset', 0, 'modified', 'qlim',[-2*pi 2*pi]);
L(4) = Link('revolute','alpha', 0,    'a',l3,  'd',0,   'offset', 0,   'modified', 'qlim',[-2*pi 2*pi]);
        

robot = SerialLink(L,'name','Phantom_x');
robot.tool = [0 0 1 l4;
              1 0 0 0;
              0 1 0 0;
              0 0 0 1];
maximo = [-0.800 0.800 -0.800 0.800 0 0.800];
pose_1 = [pi/2 pi/4 -pi/2 -pi/4];
pose_2 = [0 -pi/2 0 -pi/2];
robot.plot(pose_1,'workspace', maximo,'noa','view',[30 30]);
robot.teach;       
%% Determinación del plano de trabajo del robot:
close;
clc;
q = solucion([0.0 0.256 0.0 0.0 0]);
robot.plot(q);
robot.teach;
%% 
q1 = deg2rad(-128);
q2 = deg2rad(-64.29);
q3 = deg2rad(-21.08);
q4 = deg2rad(-94.65);
robot.fkine([q1 q2 q3 q4])


%% Graficas del plano de trabajo del robot:

r = 0.206;

n = 1000;


t = linspace(0,2*pi,n);

x_d = -0.125 + r*sin(t);
y_d = 0 + r*cos(t);

x_i = 0.125 + r*sin(t);
y_i = 0 + r*cos(t);


line(x_d,y_d,'Color','red','LineWidth',1.5)
line(x_i,y_i,'Color','blue','LineWidth',1.5)
grid on;
hold on;
plot(-0.125,0,'r*');
plot(0.125,0,'b*');

title('Ubicación de los robots','FontSize',14);
xlabel('Distancia [m]','FontSize',12);
ylabel('Distancia [m]','FontSize',12);

legend('Robot_{izquierdo}','Robot_{derecho}');

axis equal

%% 


r = 0.256;

n = 1000;


t = linspace(0,2*pi,n);

x_d = 0 + r*sin(t);
y_d = 0 + r*cos(t);


line(x_d,y_d,'Color','red','LineWidth',1.5)



%%  Inicio del nodo ros
rosinit;

%% Publicadores a los controladores de las articulaciones del robot de la izquierda:

publicador_robot_izquierda_joint_1 = rospublisher('/robot_izquierda/joint1_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot izquierdo de la articulación 1 creado...');

publicador_robot_izquierda_joint_2 = rospublisher('/robot_izquierda/joint2_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot izquierdo de la articulación 2 creado...');

publicador_robot_izquierda_joint_3 = rospublisher('/robot_izquierda/joint3_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot izquierdo de la articulación 3 creado...');

publicador_robot_izquierda_joint_4 = rospublisher('/robot_izquierda/joint4_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot izquierdo de la articulación 4 creado...');

publicador_robot_izquierda_gripper = rospublisher('/robot_izquierda/gripper_position_controller/command','std_msgs/Float64MultiArray');
pause(1);
disp('Publicador del gripper del robot izquierdo creado...');

%% Creación del mensaje de articulaciones del robot de la izquierda:

robot_izquierda_articulacion_1 = rosmessage(publicador_robot_izquierda_joint_1);
robot_izquierda_articulacion_2 = rosmessage(publicador_robot_izquierda_joint_2);
robot_izquierda_articulacion_3 = rosmessage(publicador_robot_izquierda_joint_3);
robot_izquierda_articulacion_4 = rosmessage(publicador_robot_izquierda_joint_4);
robot_izquierda_gripper = rosmessage(publicador_robot_izquierda_gripper);

%% Configuración y envio de datos de las articulaciones del robot de la izquierda:

robot_izquierda_articulacion_1.Data = deg2rad(180);
robot_izquierda_articulacion_2.Data = deg2rad(-90);
robot_izquierda_articulacion_3.Data = deg2rad(90);
robot_izquierda_articulacion_4.Data = deg2rad(90);
cierre_gripper = 0;
if cierre_gripper
    robot_izquierda_gripper.Data = [0.01,0.01];
else
    robot_izquierda_gripper.Data = [0.0,0.0];
end

send(publicador_robot_izquierda_joint_1,robot_izquierda_articulacion_1);
send(publicador_robot_izquierda_joint_2,robot_izquierda_articulacion_2);
send(publicador_robot_izquierda_joint_3,robot_izquierda_articulacion_3);
send(publicador_robot_izquierda_joint_4,robot_izquierda_articulacion_4);
send(publicador_robot_izquierda_gripper,robot_izquierda_gripper);


%% Publicadores a los controladores de las articulaciones del robot de la derecha:

publicador_robot_derecha_joint_1 = rospublisher('/robot_derecha/joint1_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot derecha de la articulación 1 creado...');

publicador_robot_derecha_joint_2 = rospublisher('/robot_derecha/joint2_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot derecha de la articulación 2 creado...');

publicador_robot_derecha_joint_3 = rospublisher('/robot_derecha/joint3_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot derecha de la articulación 3 creado...');

publicador_robot_derecha_joint_4 = rospublisher('/robot_derecha/joint4_position_controller/command','std_msgs/Float64');
pause(1);
disp('Publicador del robot derecha de la articulación 4 creado...');

publicador_robot_derecha_gripper = rospublisher('/robot_derecha/gripper_position_controller/command','std_msgs/Float64MultiArray');
pause(1);
disp('Publicador del gripper del robot derecha creado...');

%% Creación del mensaje de articulaciones del robot de la derecha:

robot_derecha_articulacion_1 = rosmessage(publicador_robot_derecha_joint_1);
robot_derecha_articulacion_2 = rosmessage(publicador_robot_derecha_joint_2);
robot_derecha_articulacion_3 = rosmessage(publicador_robot_derecha_joint_3);
robot_derecha_articulacion_4 = rosmessage(publicador_robot_derecha_joint_4);
robot_derecha_gripper = rosmessage(publicador_robot_derecha_gripper);

%% Configuración y envio de datos de las articulaciones del robot de la derecha:

robot_derecha_articulacion_1.Data = deg2rad(0);
robot_derecha_articulacion_2.Data = deg2rad(-90);
robot_derecha_articulacion_3.Data = deg2rad(0);
robot_derecha_articulacion_4.Data = deg2rad(-90);
cierre_gripper = 0;
if cierre_gripper
    robot_derecha_gripper.Data = [0.01,0.01];
else
    robot_derecha_gripper.Data = [0.0,0.0];
end

send(publicador_robot_derecha_joint_1,robot_derecha_articulacion_1);
send(publicador_robot_derecha_joint_2,robot_derecha_articulacion_2);
send(publicador_robot_derecha_joint_3,robot_derecha_articulacion_3);
send(publicador_robot_derecha_joint_4,robot_derecha_articulacion_4);
send(publicador_robot_derecha_gripper,robot_derecha_gripper);

%% Suscriptor de las articulaciones del robot de la izquierda:

suscriptor_R_Izquierdo = rossubscriber('/robot_izquierda/joint_states');
pause(1);


%% Verificación de la posición actual de las articulaciones:

actual_configuracion_R_izquierdo = receive(suscriptor_R_Izquierdo,3);
disp(actual_configuracion_R_izquierdo.Position);

%% Verificación de la camara:

imgsub = rossubscriber('/camera/image_data');
img = receive(imgsub);
figure
imshow(readImage(img))
tic
while toc<6
    img = receive(imgsub);
    imshow(readImage(img));
end


%% Probando el delete_model service de gazebo desde matlab:

delete_service = rossvcclient('/gazebo/delete_model');
mensaje_delete = rosmessage(delete_service);

%% enviando en delte model
mensaje_delete.ModelName = 'tuerca_1';
envio_delte = call(delete_service,mensaje_delete,'Timeout',3);

%%
mensaje_delete.ModelName = 'tornillo_1';
envio_delte = call(delete_service,mensaje_delete,'Timeout',3);

%% Probando el set_model_propetis service de gazebo desde matlab:

set_service = rossvcclient('/gazebo/set_model_state');
mensaje_seteo = rosmessage(set_service);

%% Enviando un seteo:
mensaje_seteo.ModelState.ModelName = 'tuerca_4';
mensaje_seteo.ModelState.ReferenceFrame = 'world';
mensaje_seteo.ModelState.Pose.Position.X = 1;
mensaje_seteo.ModelState.Pose.Position.Y = 0;
mensaje_seteo.ModelState.Pose.Position.Z = 0;
envio_seteo = call(set_service,mensaje_seteo);

%% Problando los servicios de las propiedades fisicas:
fisica_service = rossvcclient('/gazebo/set_physics_properties');
mensaje_seteo_fisico = rosmessage(fisica_service);

%% enviando configuración de fisica
mensaje_seteo_fisico.Gravity.X = 0;
mensaje_seteo_fisico.Gravity.Y = 0;
mensaje_seteo_fisico.Gravity.Z = -9.81;
enviar_seteo_fisico = call(fisica_service,mensaje_seteo_fisico);

%% Pausar la fisica
fisica_pause_service = rossvcclient('/gazebo/pause_physics');
mensaje_pausa_fisica = rosmessage(fisica_pause_service);
%%
fisica_despausada_service = rossvcclient('/gazebo/unpause_physics');
mensaje_despausa_fisica = rosmessage(fisica_despausada_service);

%% Módelo cinemático inverso del robot phanton X pincher 4R: 
function q = solucion(data)

x = data(1);
y = data(2);
z = data(3);
phi = deg2rad(data(4));
l1 = 0.135875;
l2 = 0.107;
l3 = 0.107;
l4 = 0.091;
elbow = data(5); % 1 Para codo abajo, 0 para codo arriba.

q = zeros(1,4);x = data(1);
y = data(2);
z = data(3);
phi = deg2rad(data(4));
l1 = 0.135875;
l2 = 0.107;
l3 = 0.107;
l4 = 0.091;
elbow = data(5); % 1 Para codo abajo, 0 para codo arriba.

q = zeros(1,4);
q(1) = atan2(y,x);
x_0 = sqrt(x.^2 + y.^2) - l4 * cos(phi);
z_0 = (z-l1) - l4 * sin(phi);

num = x_0.^2 + z_0.^2 - l2.^2 - l3.^2;
den = 2*l2*l3;
D = num./den;
flag = (D<=1);

if flag
    q(3) = atan2(-sqrt(1-D.^2),D);
    if elbow
        q(3) = atan2(sqrt(1-D.^2),D);
    end
    
    q(2) = -pi/2 + (atan2(z_0,x_0) - atan2(l3*sin(q(3)), l2+l3*cos(q(3))));
    q(4) = phi - pi/2 - q(2) - q(3);
       
    
else
    warning('No se hallo una solución real');
    q = NaN(1,4);    
end
q(1) = atan2(y,x);
x_0 = sqrt(x.^2 + y.^2) - l4 * cos(phi);
z_0 = (z-l1) - l4 * sin(phi);

num = x_0.^2 + z_0.^2 - l2.^2 - l3.^2;
den = 2*l2*l3;
D = num./den;
flag = (D<=1);

if flag
    q(3) = atan2(-sqrt(1-D.^2),D);
    if elbow
        q(3) = atan2(sqrt(1-D.^2),D);
    end
    
    q(2) = -pi/2 + (atan2(z_0,x_0) - atan2(l3*sin(q(3)), l2+l3*cos(q(3))));
    q(4) = phi - pi/2 - q(2) - q(3);
       
    
else
    warning('No se hallo una solución real');
    q = NaN(1,4);    
end



end