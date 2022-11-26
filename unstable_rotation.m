function unstable_rotation
   % Angular momentum of rotating rigid body
   % source code: Cleve's Corner Blog, 8/10/2015
   % https://blogs.mathworks.com/cleve/2015/08/10/tumbling-box-ode/#respond
   % Modified by Agnes Kovesdy, 11/26/2022

   function ydot = momentum(t,y,y0)
       i_1 = 1;
       i_2 = 1/2;
       i_3 = 1/3;
       ydot = [ (1/i_3 - 1/i_2)*y(2)*y(3); (1/i_1 - 1/i_3)*y(1)*y(3); 
                (1/i_2 - 1/i_1)*y(1)*y(2)];
   end

   % trajectories for z > 0
   function sphere_points_1 = points_1(x, y, z)
       x = 0;
       for y = 0.0:0.05:1.0
           z = sqrt(1-y^2);
           [t,y] = ode45(@momentum,[0,100],[x y z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
       for y = 0.88:0.02:1.0
           z = sqrt(1-y^2);
           [t,y] = ode45(@momentum,[0,100],[x y z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
   end

   % trajectories for z < 0
   function sphere_points_2 = points_2(x, y, z)
       x = 0;
       for y = 0.0:0.05:1.0
           z = sqrt(1-y^2);
           [t,y] = ode45(@momentum,[0,100],[x y -z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
       for y = 0.88:0.02:1.0
           z = sqrt(1-y^2);
           [t,y] = ode45(@momentum,[0,100],[x y -z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
   end

   % trajectories for x > 0
   function sphere_points_3 = points_3(x, y, z)
       z = 0;
       for x = 0.0:0.05:1.0
           y = sqrt(1-x^2);
           [t,y] = ode45(@momentum,[0,100],[x y z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
   end

   % trajectories for x < 0
   function sphere_points_4 = points_4(x, y, z)
       z = 0;
       for x = 0.0:0.05:1.0
           y = sqrt(1-x^2);
           [t,y] = ode45(@momentum,[0,100],[-x y z]);
           line(y(:,1),y(:,2),y(:,3),'color','black');
       end
   end

   function [val,isterm,dir] = gstop(t,y,y0)
      d = y - y0;
      v = momentum(t,y);
      val = d'*v;
      isterm = 1;
      dir = 1;
   end

   opt = odeset('RelTol',1.e-6,'events',@gstop);
   clf
   [X,Y,Z] = sphere(36);
   h = surface(X,Y,Z);
   view(3)
   zoom(2)
   shading interp
   axis equal
   axis vis3d
   axis off
   set(gca,'xdir','rev','ydir','rev')
   dkblue = [0 0 .5];
   line(0,0,1.01,'marker','.','color',dkblue,'markersize',20)
   line(0,1.01,0,'marker','.','color',dkblue,'markersize',20)
   line(1.01,0,0,'marker','.','color',dkblue,'markersize',20)
   points_1(0, 0, 0)
   points_2(0, 0, 0)
   points_3(0, 0, 0)
   points_4(0, 0, 0)
end
