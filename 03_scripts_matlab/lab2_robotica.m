%% Laboratorio #2 Robótica 202-II.
% Por: 
%
% Leonardo Fabio Mercado Benítez.
%
% Cód: 25481090.
%
% C.C: 1.016.050.737
%
% Script para el desarrollo del laboratorio 2 de Robótica 2020-II

clc;
clear;
close all;


%% Inicialización del nodo matlab.
rosinit

%% Creación del Publicador e instancia del mensaje.
velPub = rospublisher("/turtle1/cmd_vel","geometry_msgs/Twist");
velMsg = rosmessage(velPub);

%% Creación y envio del mensaje.
velMsg.Linear.X = 1;
send(velPub,velMsg)
pause(1)

%% Subscriptor del tópico de la pose del nodo /turtle1
pose_turtle_1 = rossubscriber("/turtle1/pose");
%% Solicitar último mensaje del tópico /turtle1/pose a través del Subscriptor
disp(pose_turtle_1.LatestMessage)
%% Modificación de la pose de /turtle1
pose_service = rossvcclient("/turtle1/teleport_absolute");
mensaje_pose_servicio = rosmessage(pose_service);
mensaje_pose_servicio.X = 6;
mensaje_pose_servicio.Y = 3;
mensaje_pose_servicio.Theta = 90;

envio_mensaje_de_servicio = call(pose_service,mensaje_pose_servicio,'Timeout',3);
disp(envio_mensaje_de_servicio)



