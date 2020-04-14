clear; clc;

b6 = -0.001; % Impact check depending on b6

Vt = 20; % target lane speed = this becomes final velocit 
TH = 1;
T = 6;   % lane change duration

tic 
% Initial Boundary Condition of LC
xi = 0 ; % Longitudinal position 
xidot = 22.5; % Longitudinal velocity 
xiddot = 0; % Longitudinal acceleration
yi= 1.75; % Lateral position 
yidot = 0; % Lateral velocity
yiddot = 0; % Lateral acceleration
xinitial = [xi xidot xiddot yi yidot yiddot];

% Target leader X axis at time t
xlidot = Vt; % Target leader moving at a constant velocity
xli = 0.2*xlidot; % target leader poision at first

% Final Boundary Condition of LC
xf = xli + xlidot*T - TH*xlidot;
xfdot = Vt; 
xfddot = 0;
yf = 5.25; 
yfdot = 0; 
yfddot = 0;
xfinal = [xf xfdot xfddot yf yfdot yfddot];


% X polynomial
A = [T^3 T^4 T^5;3*T^2 4*T^3 5*T^4;6*T 12*T^2 20*T^3];
m = inv(A)*[xf-(1/2)*xfddot*T^2-xidot*T-xi; xfdot-xfddot*T-xidot; xfddot-xiddot]; % [m3; m4; m5]
n = inv(A)*[-T^6;-6*T^5;-30*T^4]; % [n3; n4; n5]

m0 = xi;
m1 = xidot;
m2 = xiddot/2;
m3 = m(1);
m4 = m(2);
m5 = m(3);

n0 = 0;
n1 = 0;
n2 = 0;
n3 = n(1);
n4 = n(2);
n5 = n(3);

b5 = m5+n5*b6;
b4 = m4+n4*b6;
b3 = m3+n3*b6;
b2 = m2+n2*b6;
b1 = m1+n1*b6;
b0 = m0+n0*b6;

% Y polynomial
sym('x1');
sym('x2');
x1 = 0.0; % LC begins at t = 0
x2 = T; % LC ends at t = T 
        
syms k5 k4 k3 k2 k1 k0
sols=solve(k5*x1^5+k4*x1^4+k3*x1^3+k2*x1^2+k1*x1+k0==yi,...
    k5*5*x1^4+k4*4*x1^3+k3*3*x1^2+k2*2*x1+k1==yidot,...
    k5*20*x1^3+k4*12*x1^2+k3*6*x1+k2*2==yiddot,...
    k5*x2^5+k4*x2^4+k3*x2^3+k2*x2^2+k1*x2+k0==yf,...
    k5*5*x2^4+k4*4*x2^3+k3*3*x2^2+k2*2*x2+k1==yfdot,...
    k5*20*x2^3+k4*12*x2^2+k3*6*x2+k2*2==yfddot);

k5=double(sols.k5);
k4=double(sols.k4);
k3=double(sols.k3);
k2=double(sols.k2);
k1=double(sols.k1);
k0=double(sols.k0);

coefficientX = [b6 b5 b4 b3 b2 b1 b0]; % X poly : 6th order poly 
coefficientY = [0 k5 k4 k3 k2 k1 k0]; % Y poly : 5th order


xF = [-20 yf Vt]; % States of Platoon Head

da = 0; % initial accl
tt = 0;
tm = 1;
dt = 0.2; % time stamps
impact = 0;



while 1

    x=b6.*(tt.^6)+(b5).*(tt.^5)+(b4).*(tt.^4)+(b3).*(tt.^3)+(b2).*(tt.^2)+(b1).*(tt.^1)+(b0).*(tt.^0);
    xv=(6*b6).*(tt.^5)+5*(b5).*(tt.^4)+4*(b4).*(tt.^3)+3*(b3).*(tt.^2)+2*(b2).*(tt.^1)+(b1).*(tt.^0);
    xa=(30*b6).*(tt.^4)+(20*(b5)).*(tt.^3)+(12*(b4)).*(tt.^2)+(6*(b3)).*(tt.^1)+(2*(b2)).*(tt.^0);
    tn = tt + dt;
    xan=(30*b6).*(tn.^4)+(20*(b5)).*(tn.^3)+(12*(b4)).*(tn.^2)+(6*(b3)).*(tn.^1)+(2*(b2)).*(tn.^0);
    
    
    [xF(3) xv]
    
%     if tt > tm - 0.001 % after reaction time (do we need to consider this in connected environment)
    if tt > 0 % no reaction time
        if xF(3) > xv % platoon head > lane change car
            
                Np    = 2;  % Prediction steps
                da1   = 0.5;  % : constraints ex) max accl : 0.4 - How about relating this to the difference btw TH and current distance? Make it big
                A22   = [ones(1,Np);(-1)*ones(1,Np);eye(Np);(-1)*eye(Np)]; %need constraints for accleration
                b22   = [da1*Np;da1*Np;da1*ones(1,Np)';da1*ones(1,Np)']; % moving direction constraints
                
                % Getting Optimal Input Increments 
                a0 = zeros(1,Np)'; 
                g = @(da_set)myfunminTHCC_new(Vt,TH,da_set,da,[x 5.25 xv xa xan],xF,Np,115);
                da_set = fmincon(g,a0,A22,b22);

                disp('Deceleration')


        else 
                tk = tt;
                Np = 10; 

                    % LC's accl in Prediction Horizon Np is used (V2V condition?)
                    for j = 1:Np
                        xa(j) = (30*b6).*(tk.^4)+(20*(b5)).*(tk.^3)+(12*(b4)).*(tk.^2)+(6*(b3)).*(tk.^1)+(2*(b2)).*(tk.^0);
                        tk = tk + dt;                     
                    end

                da2   = 1.; 
                A22   = [ones(1,Np);(-1)*ones(1,Np);eye(Np);(-1)*eye(Np)]; 
                b22   = [da2*Np;da2*Np;da2*ones(1,Np)';da2*ones(1,Np)'];

                % Getting Optimal Input Increments
                a0 = zeros(1,Np)'; 
                g = @(da_set)myfunminCC2_new(Vt,da_set,da,xa,xF,Np);

                da_set = fmincon(g,a0,A22,b22);

                disp('Recovering the speed')

        end        
               
        disp(['ACCL Change: ',num2str(da_set(1))])
        da = da + da_set(1); 
        
        if  ( da > 0 ) || ( tt > 6.3 ) %% shockwave in acceleration
            
            disp(['First Positive ACCL = ',num2str(da), ' > 0 from the time is ',num2str(tt)])
            disp('Starting to accelerate!')
            break;
        end
        
    end
    
    
    disp(['ACCL =',num2str(da), ' at time ',num2str(tt)])
    
    if tt ~= 0  
        [t,x6]=ode45(@(t,x)OthVehicle(t,x,da), [tt tt+dt], xF);
        n3 = size(x6,1);
        xF = x6(n3,:);
    end
        
    tt = tt + dt; 
 
    impact = impact + da;

end

impact
toc
