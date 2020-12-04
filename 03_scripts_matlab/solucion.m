function q = solucion(data)

x = data(1);
y = data(2);
z = data(3);
phi = deg2rad(data(4));
l1 = 0.135875;
l2 = 0.107;
l3 = 0.107;
l4 = 0.091;
elbow = 0;

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