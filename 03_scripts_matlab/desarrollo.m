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
%l4 = 0.066;


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
pose_1 = [0 pi/4 -pi/2 -pi/4];
robot.plot(pose_1,'workspace', maximo,'noa','view',[30 30]);
robot.teach;       
%% Determinación del plano de trabajo del robot:
close;
clc;
q = solucion([0.0 0.206 0.03 -90 0]);
robot.plot(q);
robot.teach;

%% Graficas del plano de trabajo del robot:
r = 0.206;

n = 1000;


t = linspace(0,2*pi,n);

x_d = -0.103 + r*sin(t);
y_d = 0 + r*cos(t);

x_i = 0.103 + r*sin(t);
y_i = 0 + r*cos(t);


line(x_d,y_d,'Color','red','LineWidth',1.5)
line(x_i,y_i,'Color','blue','LineWidth',1.5)
grid on;
hold on;
plot(-0.0515,0,'r*');
plot(0.0515,0,'b*');

title('Ubicación de los robots en función de su alcanze máximo','FontSize',14);
xlabel('Distancia [m]','FontSize',12);
ylabel('Distancia [m]','FontSize',12);

legend('Robot_{izquierdo}','Robot_{derecho}');

axis equal




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



end