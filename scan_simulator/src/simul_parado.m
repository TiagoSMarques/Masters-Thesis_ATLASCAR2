
%% Laser Scans
clear all;

syms X Y Z x_i alpha phi h;

Y=(X-x_i)/tan(phi);
Z=h-(X-x_i)*tan(alpha);

syms x_p y_p z_p a b c r;
rand_point=[x_p,y_p,z_p];
normal=[a,b,c];


%Plane inequality
plane = (x_p-X)*a+(y_p-Y)*b+(z_p-Z)*c == 0; % pretty(plane)
%Sphere inequality
sphere= (X-x_p)^2+(Y-y_p)^2+(Z-z_p)^2 <= r^2; % pretty(sphere)

%Solve plane intersection and create a function
solx = solve(plane,X);
Func_X=matlabFunction(solx);
%Sphere intersection and create function
ss_sphere=(y_p + (x_i - (a*x_p - c*(h - z_p + x_i*tan(alpha)) + b*(y_p + x_i/tan(phi)))/(a + b/tan(phi) - c*tan(alpha)))/tan(phi))^2 + (h - z_p + tan(alpha)*(x_i - (a*x_p - c*(h - z_p + x_i*tan(alpha)) + b*(y_p + x_i/tan(phi)))/(a + b/tan(phi) - c*tan(alpha))))^2 + (x_p - (a*x_p - c*(h - z_p + x_i*tan(alpha)) + b*(y_p + x_i/tan(phi)))/(a + b/tan(phi) - c*tan(alpha)))^2;
Func_sphere=matlabFunction(ss_sphere);

%% Determining intersection points
close all;

%Ajustable parameters-------------------------------------------------
% tic

%Initializing---------------------------------------------------------
pp = linspace(35.0/180*pi, 145.0/180.0*pi, 220);
aa=linspace(1,4,4);

% for var_h = linspace(0.2,0.8,7)

% ah=axes;
Sol_points=zeros(3,1,numel(aa));
Dens_res=zeros(2,1);
iter=0;

for ii = linspace(pi/10,pi/2,3) %20 graus
% for xx = linspace (5,30,50)
% for var_ang = linspace(0,5,20) %20 graus
figure(1);
% ii=0;
var_h=0.4;
var_ang=0.6;
xx=12;

num_pontos=0;    
iter=iter+1;
coord_sensor=[0,0,var_h];
centro_rand=[xx,1.5,0.2];
% norm_rand=[0,-sin(ii),cos(ii)]; % para planos em y-z
% norm_rand=[-sin(ii),0,cos(ii)]; %para planos em x-z
norm_rand=[-sin(ii),cos(ii),0]; %para planos em x-y


% norm_rand=norm_rand1/norm(norm_rand1)
raio=1;

% Drawing the plane --------------------------------------------------
w = null(norm_rand); % Find two orthonormal vectors which are orthogonal to v
% [P,Q] = meshgrid(-30:74:44,-centro_rand(3):0.15:-centro_rand(3)+0.15); % Provide a gridwork (you choose the size)
[P,Q] = meshgrid(-30:60:30,-20:40:20);
plane_X = centro_rand(1)+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
plane_Y = centro_rand(2)+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
plane_Z = centro_rand(3)+w(3,1)*P+w(3,2)*Q;
surf(plane_X,plane_Y,plane_Z,'FaceColor',[1 0.4 0.6]); %'FaceAlpha',0.8
hold on;
   
%Drawing the center point --------------------------------------------
XX=linspace(coord_sensor(1),coord_sensor(1)+35);
scatter3(centro_rand(1),centro_rand(2),centro_rand(3),300,'.g')
hold on;

%Drawing the circle --------------------------------------------------
% Original points, original plane
t = linspace(0,2*pi);
x_circ = cos(t)*raio;
y_circ = sin(t)*raio;
z_circ = 0*t*raio;

pnts = [x_circ;y_circ;z_circ];
% unit normal for original plane
n0 = [0;0;1]; 
n0 = n0/norm(n0); 
% unit normal for plane to rotate into 
% plane is orthogonal to n1... given by equation n1(1)*x + n1(2)*y + n1(3)*z = 0
n1 = norm_rand; 
n1 = n1/norm(n1); 
% theta is the angle between normals
cc = dot(n0,n1) / ( norm(n0)*norm(n1) ); % cos(theta)
s = sqrt(1-cc*cc);                        % sin(theta)
u = cross(n0,n1) / ( norm(n0)*norm(n1) ); % rotation axis...
u = u/norm(u); % ... as unit vector
C = 1-cc;
% the rotation matrix
R = [u(1)^2*C+cc, u(1)*u(2)*C-u(3)*s, u(1)*u(3)*C+u(2)*s
    u(2)*u(1)*C+u(3)*s, u(2)^2*C+cc, u(2)*u(3)*C-u(1)*s
    u(3)*u(1)*C-u(2)*s, u(3)*u(2)*C+u(1)*s, u(3)^2*C+cc];
% Rotated points
newPnts = R*pnts;

plot3(newPnts(1,:)+centro_rand(1),newPnts(2,:)+centro_rand(2),newPnts(3,:)+centro_rand(3),'k')
hold on;

% Calculating intersections-------------------------------------------
    for ind_a = 1:numel(aa)
        for ind_p = 1:numel(pp)
            alph = (var_ang+(ind_a-1)*0.8)/180.0*pi;
            ph=pp(ind_p);
            %Drawing the beams;
            YY=(XX-coord_sensor(1))/tan(ph);
            ZZ = coord_sensor(3)-(XX-coord_sensor(1))*tan(alph);
%             plot3(XX,YY,ZZ);
            hold on;
            %Itersections with the plane (only for X > 0)
            sol_X=Func_X(norm_rand(1),alph,norm_rand(2),norm_rand(3),coord_sensor(3),ph,...
                    coord_sensor(1),centro_rand(1),centro_rand(2),centro_rand(3));

            sol_Y = (sol_X-coord_sensor(1))/tan(ph);

            sol_Z=coord_sensor(3)-(sol_X-coord_sensor(1))*tan(alph);
            %para vaolres positivos e acima do ground
            if sol_X >=0 && sol_Z>=0
                %verifica se o ponto esta dentro da esfera
                result=Func_sphere(norm_rand(1),alph,norm_rand(2),norm_rand(3),coord_sensor(3),ph,...
                    coord_sensor(1),centro_rand(1),centro_rand(2),centro_rand(3));

                if result <= raio^2
                    Sol_points(1:3,ind_p,ind_a)=[sol_X;sol_Y;sol_Z];
                    num_pontos=num_pontos+1;
                end
            end
        end

        %Drawing intersection points
        scatter3(Sol_points(1,:,ind_a),Sol_points(2,:,ind_a),Sol_points(3,:,ind_a)...
            ,200,'.')
        hold on;grid on;
    end
    
    Area_circ=pi*raio^2;
    Densidade=num_pontos/Area_circ;
    Dens_res(1,iter)=ii*180/pi;  %graph rotation
%     Dens_res(1,iter)=xx;
%     Dens_res(1,iter)=var_ang;
    Dens_res(2,iter)=Densidade;
%     fprintf('Num of point: %d, density: %f \n',num_pontos,Densidade);
% num_pontos
end

%axis
% set(ah,'ydir','reverse')

xlim([0, 35]);ylim([-15, 15]);zlim([0, 1]);
xlabel('X');ylabel('Y');zlabel('Z');
% toc
% hold on;

s_centro=string(centro_rand);
figure(2)
a1=plot(Dens_res(1,:),Dens_res(2,:)); %graph rotation
title(sprintf('Density on a frontal plane rotating along Y [%0.1f,%0.1f,%0.1f], raio %0.2fm ',s_centro(1),s_centro(2),s_centro(3),raio))
% title(sprintf('Density on a ground plane along X [ X,%0.1f,%0.1f], radius %0.2fm ',s_centro(2),s_centro(3),raio))
% title(sprintf('Density on a vertical plane along Y based on sensor height and sensor angle in [%0.1f,%0.1f,%0.1f], raio %0.2fm',s_centro(1),s_centro(2),s_centro(3),raio))
ylabel('Density');xlabel('\alpha');
% ylim([0, 5])
% i_x=Dens_res(1,11);
% i_y=Dens_res(2,11);
% text(i_x, i_y, sprintf('h=%0.1f',var_h), 'BackgroundColor', 'w');
indd=find(linspace(0.2,0.8,7) == var_h);
% legendInfo{indd} = [sprintf('%0.1f',var_h)];
grid on;hold on;
% end

% legend(legendInfo);