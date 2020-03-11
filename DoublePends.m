function [] = DoublePend_Demo

%%  Setup User Stop

global USER_RESPONDED
USER_RESPONDED = 0;
figure
set(gcf,'WindowKeyPressFcn',@User_Responce,'WindowButtonDownFcn',@User_Responce,'DeleteFcn',@User_Responce)

while true
    clf
    USER_RESPONDED = 0;
    DoublePend_Render;
end


end


function [] = DoublePend_Render
%Sun Gravity      = 274;
%Mercury Gravity  = 3.7;
%Venus Gravity    = 8.87;
%Earth Gravity    = 9.8;
%Moon Gravity     = 1.62;
%Mars Gravity     = 3.71;
%Jupiter Gravity  = 24.92;
%Saturn Gravity   = 10.44;
%Uranus Gravity   = 8.87;
%Neptune Gravity  = 11.15;
%Pluto Gravity    = 0.58;

%%  Setup User Stop

global USER_RESPONDED

%% Parameters

Pend_1_Length = 1;
Pend_2_Length = 1;
Pend_1_Mass = 1;
Pend_2_Mass = 1.5;
Gravity = 9.8;

% Starting Parameters
DX = [pi, rand(1)*2*pi, 0, 0];
DX2 = [pi, DX(2)+.0001, 0, 0];

% Program Parameters

Count = 0;
Time = 0;
Time2 = 0;
Tail = 50;
Tail2 = 50;
Iteration = 0;
Iteration_Time = 0.2;
Tail = repmat(Pend_1_Length*[sin(DX(1)),-cos(DX(1))],[Tail,2]) + repmat([0 0 Pend_2_Length*[sin(DX(2)),-cos(DX(2))]],[Tail,1]);
Tail2 = repmat(Pend_1_Length*[sin(DX2(1)),-cos(DX2(1))],[Tail2,2]) + repmat([0 0 Pend_2_Length*[sin(DX(2)),-cos(DX2(2))]],[Tail2,1]);


%% ODE System

Double_Pendulum_ODE = @(t,DX)Double_Pendulum_System(DX, Pend_1_Length, Pend_2_Length, Pend_1_Mass, Pend_2_Mass, Gravity);
Double_Pendulum_ODE2 = @(t,DX2)Double_Pendulum_System(DX2, Pend_1_Length, Pend_2_Length, Pend_1_Mass, Pend_2_Mass, Gravity);

%% Render Prep
axis  xy equal, box on, hold on
axis(1.1*[-1 1 -1 1]*(Pend_1_Length+Pend_2_Length))
[Pend_1_Radius, Pend_2_Radius] = Pend_Mass_Radius(Pend_1_Length,Pend_2_Length,Pend_1_Mass,Pend_2_Mass);

%% Rendering

tic
while ~USER_RESPONDED
    axis  xy equal, box on, hold on
    axis(1.1*[-1 1 -1 1]*(Pend_1_Length+Pend_2_Length))
    [Time,DX] = ode45(Double_Pendulum_ODE,Time(end)*[1 0.5 0] + Iteration_Time*[0 0.5 1] ,DX(end,:)');
    [Time2,DX2] = ode45(Double_Pendulum_ODE2,Time2(end)*[1 0.5 0] + Iteration_Time*[0 0.5 1] ,DX2(end,:)');
    [Tail, Tail2, Count] = Render(Time,DX(end,:), DX2(end,:),Pend_1_Length,Pend_2_Length,Pend_1_Radius,Pend_2_Radius,Tail, Tail2, Count);
    
%     if Time(end) >= 10
%         print('Time10','-dpng');
%         break
%     end
    
    Iteration = Iteration+1;
    Iteration_Time = max(toc*(1+1/Iteration),Time(end)+2*eps);
    
end

end

function [DX] = Double_Pendulum_System(X, Pend_1_Length, Pend_2_Length, Pend_1_Mass, Pend_2_Mass, Gravity)
%% Equations

Theta_1 = X(1);
Theta_2 = X(2);
Omega_1 = X(3);
Omega_2 = X(4);

Deta_Theta_1 = Omega_1;
Deta_Theta_2 = Omega_2;
Deta_Omega_1 = (-Gravity*(2*Pend_1_Mass+Pend_2_Mass)*sin(Theta_1)-Pend_2_Mass*Gravity*sin(Theta_1-2*Theta_2)-...
    2*sin(Theta_1-Theta_2)*Pend_2_Mass*(Omega_2^2*Pend_2_Length+Omega_1^2*Pend_1_Length*cos(Theta_1-Theta_2)))...
    /(Pend_1_Length*(2*Pend_1_Mass+Pend_2_Mass-Pend_2_Mass*cos(2*Theta_1-2*Theta_2)));
Deta_Omega_2 = (2*sin(Theta_1-Theta_2)*(Omega_1^2*Pend_1_Length*(Pend_1_Mass+Pend_2_Mass)+...
    Gravity*(Pend_1_Mass+Pend_2_Mass)*cos(Theta_1)+Omega_2^2*Pend_2_Length*Pend_2_Mass*cos(Theta_1-Theta_2)))...
    /(Pend_2_Length*(2*Pend_1_Mass+Pend_2_Mass-Pend_2_Mass*cos(2*Theta_1-2*Theta_2)));

%% Store info

DX = [Deta_Theta_1;Deta_Theta_2;Deta_Omega_1;Deta_Omega_2];

end

function [Pend_1_Radius, Pend_2_Radius] = Pend_Mass_Radius(Pend_1_Length,Pend_2_Length,Pend_1_Mass,Pend_2_Mass)

%%%%%%%%%%%%%%%%
Constant = 1/10;
%%%%%%%%%%%%%%%%

%% Find Max and Min Radius
Radius_Max = max(Pend_1_Mass^(1/3), Pend_2_Mass^(1/3));
Length_Min = min(Pend_1_Length,Pend_2_Length);

%% Find Scale to render with

Scale = Constant*Length_Min/Radius_Max;

%% Calculate Pend_1_Radius and Pend_2_Radius

Pend_1_Radius = Scale*Pend_1_Mass^(1/3);
Pend_2_Radius = Scale*Pend_2_Mass^(1/3);

end

function [Tail, Tail2, Count] = Render(Time, X, X2, Pend_1_Length,Pend_2_Length, Pend_1_Radius, Pend_2_Radius, Tail, Tail2, Count)
%% Setup
cla

%% Pend 1
Theta_1 = X(1);
Theta_2 = X(2);

Pend_1_X_Coord_1 =  Pend_1_Length*sin(Theta_1); 
Pend_1_Y_Coord_1 = -Pend_1_Length*cos(Theta_1);
Pend_2_X_Coord_1 =  Pend_1_X_Coord_1 + Pend_2_Length*sin(Theta_2);
Pend_2_Y_Coord_1 =  Pend_1_Y_Coord_1 - Pend_2_Length*cos(Theta_2);

patch([0, Pend_1_X_Coord_1, Pend_2_X_Coord_1, NaN],[0, Pend_1_Y_Coord_1, Pend_2_Y_Coord_1, NaN],0,'EdgeColor','b','FaceColor','none','LineWidth',2)

%% Pend 2
Theta_1_2 = X2(1);
Theta_2_2 = X2(2);

%% Rendering Rods 1
Pend_1_X_Coord_2 =  Pend_1_Length*sin(Theta_1_2); 
Pend_1_Y_Coord_2 = -Pend_1_Length*cos(Theta_1_2);
Pend_2_X_Coord_2 =  Pend_1_X_Coord_2 + Pend_2_Length*sin(Theta_2_2);
Pend_2_Y_Coord_2 =  Pend_1_Y_Coord_2 - Pend_2_Length*cos(Theta_2_2);

patch([0, Pend_1_X_Coord_2, Pend_2_X_Coord_2, NaN],[0, Pend_1_Y_Coord_2, Pend_2_Y_Coord_2, NaN],0,'EdgeColor','r','FaceColor','none','LineWidth',2)

%% Rendering Tail of Pendulums

l = linspace(0,1,size(Tail,1)+1)';
patch([Tail(:,3);NaN],[Tail(:,4);NaN],0,'EdgeColor','c','FaceColor','none','FaceVertexAlphaData',l,'EdgeAlpha','interp','LineWidth',2);
patch([Tail2(:,3);NaN],[Tail2(:,4);NaN],0,'EdgeColor','m','FaceColor','none','FaceVertexAlphaData',l,'EdgeAlpha','interp','LineWidth',2);

Tail = [Tail(2:end,:);Pend_1_X_Coord_1,Pend_1_Y_Coord_1,Pend_2_X_Coord_1,Pend_2_Y_Coord_1];
Tail2 = [Tail2(2:end,:);Pend_1_X_Coord_2,Pend_1_Y_Coord_2,Pend_2_X_Coord_2,Pend_2_Y_Coord_2];
%% Plotting Mass of Pend_1 and Pend_2

Pend_1_Circle_Coord = zeros(2,45);
Pend_2_Circle_Coord = zeros(2,45);
Pend_1_Circle_Coord_2 = zeros(2,45);
Pend_2_Circle_Coord_2 = zeros(2,45);

for k = 0:45
    Pend_1_Circle_Coord(1, k+1) = Pend_1_X_Coord_1 + Pend_1_Radius * cos(pi/180 * 8*k);
    Pend_1_Circle_Coord(2, k+1) = Pend_1_Y_Coord_1 + Pend_1_Radius * sin(pi/180 * 8*k);
    Pend_2_Circle_Coord(1, k+1) = Pend_2_X_Coord_1 + Pend_2_Radius * cos(pi/180 * 8*k);
    Pend_2_Circle_Coord(2, k+1) = Pend_2_Y_Coord_1 + Pend_2_Radius * sin(pi/180 * 8*k);
    Pend_1_Circle_Coord_2(1, k+1) = Pend_1_X_Coord_2 + Pend_1_Radius * cos(pi/180 * 8*k);
    Pend_1_Circle_Coord_2(2, k+1) = Pend_1_Y_Coord_2 + Pend_1_Radius * sin(pi/180 * 8*k);
    Pend_2_Circle_Coord_2(1, k+1) = Pend_2_X_Coord_2 + Pend_2_Radius * cos(pi/180 * 8*k);
    Pend_2_Circle_Coord_2(2, k+1) = Pend_2_Y_Coord_2 + Pend_2_Radius * sin(pi/180 * 8*k);
end

patch(Pend_1_Circle_Coord(1,:), Pend_1_Circle_Coord(2,:) , 0, 'EdgeColor', 'b', 'FaceColor', 'b')
patch(Pend_2_Circle_Coord(1,:), Pend_2_Circle_Coord(2,:) , 0, 'EdgeColor', 'b', 'FaceColor', 'b')
patch(Pend_1_Circle_Coord_2(1,:), Pend_1_Circle_Coord_2(2,:) , 0, 'EdgeColor', 'r', 'FaceColor', 'r')
patch(Pend_2_Circle_Coord_2(1,:), Pend_2_Circle_Coord_2(2,:) , 0, 'EdgeColor', 'r', 'FaceColor', 'r')

%% Count Overhead Rotations

%Count = OverHead_Count(Tail, Count);

%% Draw Everything

title(sprintf('Press SPACE to restart simulation, Time = %0.1f',Time(end)))
drawnow


end

function [Count] = OverHead_Count(Tail, Count)

if Count == 1
    Count = 1;
end

if Tail(end,4) >= 0
    if (Tail(end,3) > 0 & Tail(end - 1 , 3) < 0)
        Count = Count + 1;
    elseif (Tail(end,3) < 0 & Tail(end - 1,3) > 0)
        Count = Count + 1;
    end
end

end

function User_Responce(~,~)

global USER_RESPONDED
USER_RESPONDED = 1;

end
