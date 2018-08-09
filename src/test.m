D = load('scan3D.txt');
odom = load('odom3D.txt');

% Remplazar 0 por NaN
D (D == 0.0) = NaN;
% radio
maxvalue = size(D,1)
r = D(1:maxvalue, 2:end);

%x, y plano cartesiano
x = zeros(size(r));
y = zeros(size(r));

for i=1:360
  x(:,i) = r(:,i)*cosd(i);
  y(:,i) = r(:,i)*sind(i);
end

x_odom = odom(1:maxvalue,2);
y_odom = odom(1:maxvalue,3);
theta = odom(1:maxvalue,4);

x3d = zeros(size(r));
y3d = zeros(size(r));
z3d = zeros(size(r));

totalang = maxvalue;
x3 = zeros(totalang, size(r,2));
y3 = zeros(totalang, size(r,2));
z3 = zeros(totalang, size(r,2));

d = 0.10;  # metros
zlid = zeros(1, size(r,2));
unos = ones(1, size(r,2));

for m=1:totalang
  ang = D(m, 1)/180*pi;
  R = [cos(ang), 0, sin(ang), 0;
       0, 1, 0, 0;
       -sin(ang), 0, cos(ang), 0;
       0, 0, 0, 1];
  T = eye(4);
  T(3,4) = d;
  # Transf del sist Lidar con respecto al sist Motor
  Tf = R * T;
  # Matriz de puntos con igual ang del motor en sist Lidar
  Plid = [x(m,:); y(m,:); zlid; unos];
  # Puntos en sist motor
  Pm = Tf * Plid;
  x3(m,:) = Pm(1,:);
  y3(m,:) = Pm(2,:);
  z3(m,:) = Pm(3,:);
end  

for  n = 1:maxvalue
  angulo = deg2rad(theta(n));
  R_frameK = [cos(angulo),-sin(angulo),0;sin(angulo),cos(angulo),0;0,0,1];
  sm = [x3(n,:);y3(n,:); z3(n,:)];
  puntos = R_frameK * sm;
  x3d(n,:) = puntos(1,:) + x_odom(n);
  y3d(n,:) = puntos(2,:) + y_odom(n);
  z3d(n,:) = puntos(3,:);
end

x3d(x3d > 0.2);
y3d(x3d > 0.2);
z3d(x3d > 0.2);
%for i=1:maxvalue
%  x3d(i,:) = x3d(i,:) + x_odom(i);
%  y3d(i,:) = y3d(i,:) + y_odom(i);
%end

figure
plot3(x3d(1:10:end), y3d(1:10:end), z3d(1:10:end), '.')
%plot3(x3d(:), y3d(:), z3d(:), '.')
xlim([-0.3,3])
xlabel('x'), ylabel('y'), zlabel('z')

#xd = x3(:); yd = y3(:); zd = z3(:);
#tetr = delaunay (xd(1:1000), yd(1:1000), zd(1:1000));

%figure
%plot(y3(:), z3(:),'.')
%ylabel('y'), zlabel('z')