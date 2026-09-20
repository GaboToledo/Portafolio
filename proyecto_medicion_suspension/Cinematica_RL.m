clear all

options = optimoptions("fsolve","FunctionTolerance",1e-9);
P1_L=[56.161;89.103]; %Chasis-Amortiguador 
P2_L =[24.4;32.1]; %Bandeja-Chasis
L1_L = norm(P1_L - P2_L);
L2 = 40.51;
L3 = 9.614;
th1 = deg2rad(360-171);
th2_range = deg2rad(70:0.1:110);
sol = zeros(0,4);  % Changed to 4 columns
Suspension_travel = [];
for i = 1:length(th2_range)
    X = th2_range(i);
    if X> pi/2
        Suspension_travel(i) = -L2*cos(X);
    elseif X<= pi/2
                Suspension_travel(i) = -L2*cos(X);
    end
end

for th2i = th2_range
    th2 = th2i;
    
    % Initial guess as a vector [th3, th4, L4]
    x0 = [deg2rad(180), deg2rad(180+178.6), 40];
    
    % Create anonymous function that passes fixed parameters
    fun = @(x) camper_system(x, th1, th2, L1_L, L2, L3);
    
    % Solve the system
    x_sol = fsolve(fun, x0, options);
    
    % Store solution [th2, th3, th4, L4]
    sol = [sol; th2, x_sol(1), x_sol(2), x_sol(3)];
end

sol(:,1) = rad2deg(sol(:,1));  % Convert th2 to degrees
sol(:,2) = rad2deg(sol(:,2));  % Convert th3 to degrees
sol(:,3) = rad2deg(sol(:,3));  % Convert th4 to degrees
% L4 stays in original units

solsuave = movmean(sol, 100, 1);

for i = 1:4  % Changed to 4 since we have 4 columns
    subplot(2,2,i)
    plot(Suspension_travel, solsuave(:,i))
    if i == 1
        ylabel('th2 (deg)')
    elseif i == 2
        ylabel('Camber (deg)')
    elseif i == 3
        ylabel('th4 (deg)')
    else
        ylabel('L4')
    end
    xlabel('Suspension travel (cm)')
end



% Function that fsolve can use (single vector input)
function F = camper_system(x, th1, th2, L1, L2, L3)
    th3 = x(1);
    th4 = x(2);
    L4 = x(3);
    
    % Equations to solve
    fc = L1*cos(th1) + L2*cos(th2) + L3*cos(th3) + L4*cos(th4);
    fs = L1*sin(th1) + L2*sin(th2) + L3*sin(th3) + L4*sin(th4);
    sumth = th3 + deg2rad(178.6) - th4;
    
    % Return as vector (should be zero at solution)
    F = [fc; fs; sumth];
end
