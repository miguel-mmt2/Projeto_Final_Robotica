%===========================================
%           Laboratory class #1
%               Exercise 1
%             (With Toolbox)
%
%
% Nome: Ângelo da Rocha Rodrigues
% Número: 2021236348
%
% Nome: Miguel Meireles Teixeira
% Número: 2021217493
%===========================================

clc; clear; close all;

% Tamanho dos eixos:
axis_scale_factor = 200;
Limit_Axis = axis_scale_factor * [-1 1 -1 1 -1 1];

C= [200, 200, 100];
r = 100;
alpha_velocity = pi/2;
alpha= 0;
iterationsTime = 0.1;
% Variáveis:
syms t1 t2 t3 t4 t5 t6

offset1=0; offset2=-pi/2; offset3=-pi/2; offset4=0; offset5=0; offset6=0;

d1=243.3;    d2=0;   d3=0;  d4=227.6;   d5=0;  d6=61.5;

a1=0;  a2=200;   a3=87;  a4=0;   a5=0;  a6=0;

alpha1=-pi/2;    alpha2=pi;     alpha3=pi/2;    alpha4=pi/2;    alpha5=-pi/2;    alpha6=0;



DH_Matrix = [t1+offset1      d1      a1      alpha1
            t2+offset2       d2      a2      alpha2
            t3+offset3       d3      a3      alpha3
            t4+offset4       d4      a4      alpha4
            t5+offset5       d5      a5      alpha5
            t6+offset6       d6      a6      alpha6];


% Confirm the Transformation Matrices

[Transformation_Matrices,T_final] = MGH_DH(DH_Matrix)
T_final = simplify(T_final)


% Criação Robô:
qlim=[0,2*pi];

% Criação dos Links do Robô:
L1 = Link('revolute','d', d1, 'a', a1, 'alpha',alpha1,'offset',offset1,'qlim',qlim);
L2 = Link('revolute','d', d2, 'a', a2, 'alpha',alpha2,'offset',offset2,'qlim',qlim);
L3 = Link('revolute','d', d3, 'a', a3, 'alpha',alpha3,'offset',offset3,'qlim',qlim);
L4 = Link('revolute','d', d4, 'a', a4, 'alpha',alpha4,'offset',offset4,'qlim',qlim);
L5 = Link('revolute','d', d5, 'a', a5, 'alpha',alpha5,'offset',offset5,'qlim',qlim);
L6 = Link('revolute','d', d6, 'a', a6, 'alpha',alpha6,'offset',offset6,'qlim',qlim);


% Criação do UFactory-Lite6:
UFactory_Lite6 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'UFactory-Lite6');
Auxiliar_Lite6 = SerialLink([L1 L2 L3 L4 L5 L6], 'name', 'Auxiliar-Lite6 ');

% Vizualização do Robô:

% UFactory_Lite6.plot([0 0 0 0 0 0]);    % Print dos primeiros thetas
% hold on;
% 
% P_Home = [0 0 0 0 0 0]; 
% UFactory_Lite6.teach(P_Home, 'workspace', Limit_Axis);


Transformation_Matrices(:,:, 1) = simplify(Transformation_Matrices(:,:, 1));  % T01
Transformation_Matrices(:,:, 2) = simplify(Transformation_Matrices(:,:, 2));  % T12
Transformation_Matrices(:,:, 3) = simplify(Transformation_Matrices(:,:, 3));  % T23
Transformation_Matrices(:,:, 4) = simplify(Transformation_Matrices(:,:, 4));  % T34
Transformation_Matrices(:,:, 5) = simplify(Transformation_Matrices(:,:, 5));  % T45
Transformation_Matrices(:,:, 6) = simplify(Transformation_Matrices(:,:, 6));  % T56

T_01_sym = Transformation_Matrices(:,:, 1);
T_02_sym = T_01_sym * Transformation_Matrices(:,:, 2);
T_03_sym = T_02_sym * Transformation_Matrices(:,:, 3);
T_04_sym = T_03_sym * Transformation_Matrices(:,:, 4);
T_05_sym = T_04_sym * Transformation_Matrices(:,:, 5);


% Jacobiano

P_0G = T_final(1:3,4);

Jac_v = [diff(P_0G,t1) diff(P_0G,t2) diff(P_0G,t3) diff(P_0G,t4) diff(P_0G,t5) diff(P_0G,t6)]

% são todas juntas revolute
Jac_w = [[0 0 1]' T_01_sym(1:3,3) T_02_sym(1:3,3) T_03_sym(1:3,3) T_04_sym(1:3,3) T_05_sym(1:3,3)] 

J0R = [Jac_v; 
       Jac_w];

J0R = simplify(J0R)

cartisian_velocities = [                                0;
                             -r*sin(alpha)*alpha_velocity;
             r*(cos(alpha)^2-sin(alpha)^2)*alpha_velocity;
                                                        0;
                                                        0;
                                                        0];

Pos_ini = double(UFactory_Lite6.fkine([0 0 0 0 0 0]))
%% 

Mask=[1 1 1 1 1 1];  % 6 degrees of freedom
estimativa_inicial=deg2rad([0,0,0,0,0,0]);


Pos_ini(1,4)= C(1);
Pos_ini(2,4)= r*cos(alpha)+ C(2);
Pos_ini(3,4)= r* cos(alpha) * sin(alpha) + C(3);

config_rads = UFactory_Lite6.ikine(Pos_ini,'q0',estimativa_inicial,'mask',Mask)


%% 

      
  
    while 1

            J0R_red_subs = eval(subs(J0R,[t1 t2 t3 t4 t5 t6],config_rads(1:6)));

        velocities = [                                    0;
                               -r*sin(alpha)*alpha_velocity;
               r*(cos(alpha)^2-sin(alpha)^2)*alpha_velocity;
                                                          0;
                                                          0;
                                                          0];
    
            vel = inv(J0R_red_subs) * velocities;


            aux_config = config_rads + ([vel(1) vel(2) vel(3) vel(4) vel(5) vel(6)] .* iterationsTime);
            
            alpha = alpha + alpha_velocity * iterationsTime;

            UFactory_Lite6.plot(config_rads,'view','y')

            config_rads = aux_config;
      
     end


