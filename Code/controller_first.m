function f = controller_first(Vdes,TH,da,a0,sv,xF,Np,r2)

dt = 0.2 ; % Sampling Time 

if xF(3) > sv(3) 
r1 = 100; 
r2 = 50; 
r3 = 500; 
end

J = 0 ;

i = 1 ;

K1(i) = sv(1); % Current States
K2(i) = sv(2);
K3(i) = sv(3);
KA(i) = sv(4);
KAN(i) = sv(5); % next acceleration

K4(i) = xF(1); % platoon leader (follower of SV)
K5(i) = xF(2);
K6(i) = xF(3);


for i = 1 : Np  % Np : Prediction Horizon 
    %% Predicted States
%     a0  = a0 + da(i) ;


    K1(i+1) = K1(i) + dt*(K3(i)) ; % LV X axis 
    K2(i+1) = K2(i) ; % LV Y axis 
    
    if Np == 2
            K3(i+1) = K3(i) + dt*(sv(5)) ; % LV X velocity 
    else
            K3(i+1) = K3(i) + dt*(sv(4)) ; % LV X velocity 
    end

    K4(i+1) = K4(i) + dt*(K6(i)) ; % FV X axis
    K5(i+1) = K5(i) ; % FV Y axis
    K6(i+1) = K6(i) + dt*(a0) ; % FV X velocity

    a0  = a0 + da(i) ;
    
    %% Cost Function : Desired moving direction given by CFC

    J = J + r1 * ( K1(i+1) - K4(i+1) - TH*K6(i+1) ).^2 + r2 * ( sv(4) - a0  ).^2 + r3 *(da(i).^2);

    
end

f = J ;







    
   