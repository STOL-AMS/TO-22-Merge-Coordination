function f = controller_second(Vdes,da,a0,xa,xF,Np)

dt = 0.2 ; % Sampling Time 

r1 = 70;
r2 = 50; 
r3 = 800; 
r4 = 200;

J = 0 ;

i = 1 ;

K1(i) = xF(1); 
K2(i) = xF(2);
K3(i) = xF(3);
KA(i) = a0;



for i = 1 : Np  % Np : Prediction Horizon 
    %% Predicted States
%     a0  = a0 + da(i) ;

    K1(i+1) = K1(i) + dt*(K3(i)) ; % SV X axis 
    K2(i+1) = K2(i) ;              % SV Y axis
    K3(i+1) = K3(i) + dt*(KA(i)) ; % SV X axis velocity

    KA(i+1)  = KA(i) + da(i); % SV X axis acceleration

    
    %% Cost Function : Desired moving direction given by CFC

    J = J + r1 * ( K3(i+1) - Vdes ).^2 + r2 * ( KA(i+1) - xa(i)  ).^2 + r3 *(da(i).^2) + r4 * ( KA(i+1) - 0  ).^2 ;

    
end

f = J ;







    
   