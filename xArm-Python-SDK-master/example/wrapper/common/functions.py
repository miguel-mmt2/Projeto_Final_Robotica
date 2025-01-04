#!/usr/bin/env python3

# Importar as Bibliotecas necessárias:
import os

import numpy as np
from numpy import sin
from numpy import cos
import matplotlib.pyplot as plt

# ============ Funções Auxiliares ============

"""
Esta função limpa o terminal dependendo do sistema operativo do utilizador
"""
def clc():
    os.system('cls' if os.name == 'nt' else 'clear')  # Limpa o terminal



"""
Esta função mostra as diferentes opções para o Menu
"""
def opcoes_menu():
    opcao = "0"
    metodo = "0"

    # Escolher a Equação:
    while (opcao!="1" and opcao!="2" and opcao !="3" and opcao!="4" and opcao!="5" and opcao!="6"):
        clc() # Limpa o Terminal

        print("===================")
        print("| [1] - Equação 1 |")
        print("| [2] - Equação 2 |")
        print("| [3] - Equação 3 |")
        print("| [4] - Equação 4 |")
        print("| [5] - Equação 5 |")
        print("| [6] - Equação 6 |")
        print("===================")
        opcao = input("Opção: ")

    # Escolher o Método:
    while (metodo!="1" and metodo!="2"):
        clc() # Limpa o Terminal

        print("======================")
        print("| [1] - Com Controlo |")
        print("| [2] - Sem Controlo |")
        print("======================")
        metodo = input("Opção: ")

    return opcao, metodo



"""
Esta função obtêm as constantes da Equação do Infinito
"""
def constantes_infinito(C, r):
    while not (C[0].isdecimal() and C[1].isdecimal() and C[2].isdecimal() and r.isdecimal()):
        clc()  # Limpa o terminal

        print("Introduza valores para o Centro e o Raio:")
        C[0] = input("Cx: ")
        C[1] = input("Cy: ")
        C[2] = input("Cz: ")
        r = input("Raio: ")

    print(f"\nOs valores são: C = [{C[0]}, {C[1]}, {C[2]}] e r = {r}")

    return C[0], C[1], C[2], r



"""
Esta função obtêm as constantes da Equação da Elipse
"""
def constantes_elipse(C, r_a, r_b):
    while not (C[0].isdecimal() and C[1].isdecimal() and C[2].isdecimal() and r_a.isdecimal() and r_b.isdecimal()):
        clc()  # Limpa o terminal

        print("Introduza valores para o Centro e o Raio:")
        C[0] = input("Cx: ")
        C[1] = input("Cy: ")
        C[2] = input("Cz: ")
        r_a = input("Raio a: ")
        r_b = input("Raio b: ")

    print(f"\nOs valores são: C = [{C[0]}, {C[1]}, {C[2]}], r_a = {r_a} e r_b = {r_b}")

    return C[0], C[1], C[2], r_a, r_b



"""
Esta função obtêm as constantes da Equação da Circunferência
"""
def constantes_circunferencia(C, r):
    while not (C[0].isdecimal() and C[1].isdecimal() and C[2].isdecimal() and r.isdecimal()):
        clc()  # Limpa o terminal

        print("Introduza valores para o Centro e o Raio:")
        C[0] = input("Cx: ")
        C[1] = input("Cy: ")
        C[2] = input("Cz: ")
        r = input("Raio: ")

    print(f"\nOs valores são: C = [{C[0]}, {C[1]}, {C[2]}] e r = {r}")

    return C[0], C[1], C[2], r



"""
    Esta função retorna as respetivas velocidades cartesianas com base na Equação
"""
def Compute_cartesian_velocity(opcao, r, r_a, r_b, alpha, alpha_velocity):
    if (opcao == "1"): # Equação do Infinito
        cartesian_velocities = np.array([   -r*sin(alpha)*alpha_velocity,
                                            r*(cos(alpha)**2-sin(alpha)**2)*alpha_velocity,
                                            0,
                                            0,
                                            0,
                                            0])
    elif(opcao == "2"): # Equação do Infinito
        cartesian_velocities = np.array([   0,
                                            -r*sin(alpha)*alpha_velocity,
                                            r*(cos(alpha)**2-sin(alpha)**2)*alpha_velocity,
                                            0,
                                            0,
                                            0])
    elif(opcao == "3"): # Equação da Elipse
        cartesian_velocities = np.array([   -r_a*sin(alpha)*alpha_velocity,
                                            r_b*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0,
                                            0])
    elif(opcao == "4"): # Equação da Elipse
        cartesian_velocities = np.array([   0,
                                            -r_a*sin(alpha)*alpha_velocity,
                                            r_b*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0])
    elif(opcao == "5"): # Equação da Circunferência
        cartesian_velocities = np.array([   -r*sin(alpha)*alpha_velocity,
                                            r*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0,
                                            0])
    elif(opcao == "6"): # Equação da Circunferência
        cartesian_velocities = np.array([   0,
                                            -r*sin(alpha)*alpha_velocity,
                                            r*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0])
    return cartesian_velocities


"""
    Esta função retorna as respetivas posições iniciais com base na Equação
"""
def Compute_Inical_Position(opcao, C, r, r_a, r_b, alpha):
    if(opcao == "1"):
        p_x = C[0] + r * cos(alpha) 
        p_y = C[1] + r * cos(alpha) * sin(alpha)
        p_z = C[2] #  Compensação por um dos robôs não ter Gripper

    elif(opcao == "2"):
        p_x = C[0] #  Compensação por um dos robôs não ter Gripper
        p_y = C[1] + r * cos(alpha)
        p_z = C[2] + r * cos(alpha) * sin(alpha)

    elif(opcao == "3"):
        p_x = C[0] + r_a * cos(alpha)
        p_y = C[1] + r_b * sin(alpha)
        p_z = C[2] #  Compensação por um dos robôs não ter Gripper

    elif(opcao == "4"):
        p_x = C[0] #  Compensação por um dos robôs não ter Gripper
        p_y = C[1] + r_a * cos(alpha)
        p_z = C[2] + r_b * sin(alpha)

    elif(opcao == "5"):
        p_x = C[0] + r * cos(alpha)
        p_y = C[1] + r * sin(alpha)
        p_z = C[2] #  Compensação por um dos robôs não ter Gripper

    elif(opcao == "6"):
        p_x = C[0] #  Compensação por um dos robôs não ter Gripper
        p_y = C[1] + r * cos(alpha)
        p_z = C[2] + r * sin(alpha)

    return p_x, p_y, p_z


   
"""
    Esta função retorna os Respetivos Roll, Pitch, Yaw com base na Equação
"""
def Compute_Roll_Pitch_Yaw(opcao):
    if (opcao == "1" or opcao == "3" or opcao == "5"): 
        Roll = np.pi
        Pitch = 0
        Yaw = 0

    elif(opcao == "2" or opcao == "4" or opcao == "6"): 
        Roll = np.pi / 2
        Pitch = - np.pi / 2
        Yaw = np.pi / 2

    return Roll, Pitch, Yaw



"""
    Esta função retorna as Posições a mudar tendo em conta a Equação escolhida no Menu
"""
def Compute_Position(opcao, T_0G):
    if (opcao == "1" or opcao == "3" or opcao == "5"): 
        p1 = T_0G[0]
        p2 = T_0G[1]

    elif(opcao == "2" or opcao == "4" or opcao == "6"): 
        p1 = T_0G[1]
        p2 = T_0G[2]

    return p1, p2


"""
    Esta função retorna as velocidades Proporcionais e Integrativas tendo em conta a Equação escolhida no Menu
"""
def Compute_PI_Velocity_Errors(opcao, v1, v2, integrative_error_v1, integrative_error_v2, J0R_red_subs):
    if (opcao == "1" or opcao == "3" or opcao == "5"): 
        prop_vel = np.linalg.inv(J0R_red_subs) @ np.array([v1, v2, 0, 0, 0, 0]).T
        vel_integrative = np.linalg.inv(J0R_red_subs) @ np.array([integrative_error_v1, integrative_error_v2, 0, 0, 0, 0]).T

    elif(opcao == "2" or opcao == "4" or opcao == "6"): 
        prop_vel = np.linalg.inv(J0R_red_subs) @ np.array([0, v1, v2, 0, 0, 0]).T
        vel_integrative = np.linalg.inv(J0R_red_subs) @ np.array([0, integrative_error_v1, integrative_error_v2, 0, 0, 0]).T

    return prop_vel, vel_integrative



"""
    Esta função retorna os plots finais (Com Controlador PI)
"""
def Final_Plot(opcao, C, r, r_a, r_b, error_1_array_plot, error_2_array_plot, p1_g_r_array_plot, p2_g_r_array_plot, config_rads_array_plot_1, config_rads_array_plot_2, config_rads_array_plot_3, config_rads_array_plot_4, config_rads_array_plot_5, config_rads_array_plot_6, vel_config_array_plot_1, vel_config_array_plot_2, vel_config_array_plot_3, vel_config_array_plot_4, vel_config_array_plot_5, vel_config_array_plot_6, cartesian_velocities_array_plot_1, cartesian_velocities_array_plot_2, iterationTime):
    clc()  # Limpa o terminal
    print("A Simulação terminou com sucesso!")
    
    # ==> Gráficos dos erros:
    plt.figure(1, figsize=(10, 6))
   
    # -> Subplot para o Error 1
    plt.subplot(2, 1, 1)
    
    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, error_1_array_plot, label='Erro x', color='red')
        plt.ylabel('Erro e_x')
    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, error_1_array_plot, label='Erro y', color='red')
        plt.ylabel('Erro e_y')

    plt.title('Evolução dos Erros ao Longo do Tempo')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    # -> Subplot para o Error 2
    plt.subplot(2, 1, 2)

    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, error_2_array_plot, label='Erro y', color='blue')
        plt.ylabel('Erro e_y')
    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, error_2_array_plot, label='Erro z', color='blue')
        plt.ylabel('Erro e_z')

    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()



    # ==> Gráfico da trajetória no plano cartesiano
    pos_1, pos_2 = Compute_Equations(opcao, C, r, r_a, r_b)

    plt.figure(2, figsize=(10, 6))
    plt.plot(pos_1, pos_2, label='Trajetória de Referência', color='red')
    plt.plot(p1_g_r_array_plot, p2_g_r_array_plot, label='Trajetória Real', color='blue')
    plt.title('Trajetória de Referência vs Trajetória Real')
    
    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.xlabel('Posição X (mm)')
        plt.ylabel('Posição Y (mm)')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.xlabel('Posição Y (mm)')
        plt.ylabel('Posição Z (mm)')

    plt.grid(True)
    plt.legend()
    plt.tight_layout()  



    # ==> Gráficos das posições das juntas
    # Gráficos das posições das juntas
    plt.figure(3, figsize=(12, 10))

    # -> Subplot para cada junta Posição
    plt.subplot(6, 1, 1)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_1, label='Junta 1', color='red')
    plt.title('Posições das Juntas ao Longo do Tempo (rad)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_2, label='Junta 2', color='orange')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 3)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_3, label='Junta 3', color='green')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 4)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_4, label='Junta 4', color='blue')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_5, label='Junta 5', color='purple')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, config_rads_array_plot_6, label='Junta 6', color='brown')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()



    # ==> Gráficos das Velocidades das juntas
    # Gráficos das Velocidades das juntas
    plt.figure(4, figsize=(12, 10))

    # Subplot para cada junta Velocidade
    plt.subplot(6, 1, 1)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_1, label='Junta 1', color='red')
    plt.title('Velocidade das Juntas ao Longo do Tempo (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_2, label='Junta 2', color='orange')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 3)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_3, label='Junta 3', color='green')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 4)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_4, label='Junta 4', color='blue')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_5, label='Junta 5', color='purple')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, vel_config_array_plot_6, label='Junta 6', color='brown')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()



    # ==> Gráficos das Velocidades Cartesianas
    plt.figure(5, figsize=(10, 6))

    # Subplot para Cartesiana Velocidade
    plt.subplot(6, 1, 1)

    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, cartesian_velocities_array_plot_1, label='v_x', color='red')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, cartesian_velocities_array_plot_1, label='v_y', color='red')
    
    plt.title('Velocidade Cartesianas ao Longo do Tempo (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Velocidade (rad/s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)

    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, cartesian_velocities_array_plot_2, label='v_y', color='red')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(error_1_array_plot))) * iterationTime, cartesian_velocities_array_plot_2, label='v_z', color='red')
    
    plt.ylabel('Velocidade (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()
    

    plt.show()



"""
    Esta função retorna os plots finais (Sem Controlador PI)
"""
def Final_Plot_2(opcao, C, r, r_a, r_b, p1_g_r_array_plot, p2_g_r_array_plot, config_rads_array_plot_1, config_rads_array_plot_2, config_rads_array_plot_3, config_rads_array_plot_4, config_rads_array_plot_5, config_rads_array_plot_6, vel_config_array_plot_1, vel_config_array_plot_2, vel_config_array_plot_3, vel_config_array_plot_4, vel_config_array_plot_5, vel_config_array_plot_6, cartesian_velocities_array_plot_1, cartesian_velocities_array_plot_2, iterationTime):
    clc()  # Limpa o terminal
    print("A Simulação terminou com sucesso!")
    
    # ==> Gráfico da trajetória no plano cartesiano
    pos_1, pos_2 = Compute_Equations(opcao, C, r, r_a, r_b)

    plt.figure(1, figsize=(10, 6))
    plt.plot(pos_1, pos_2, label='Trajetória de Referência', color='red')
    plt.plot(p1_g_r_array_plot, p2_g_r_array_plot, label='Trajetória Real', color='blue')
    plt.title('Trajetória de Referência vs Trajetória Real')
    
    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.xlabel('Posição X (mm)')
        plt.ylabel('Posição Y (mm)')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.xlabel('Posição Y (mm)')
        plt.ylabel('Posição Z (mm)')

    plt.grid(True)
    plt.legend()
    plt.tight_layout()  



    # ==> Gráficos das posições das juntas
    # Gráficos das posições das juntas
    plt.figure(2, figsize=(12, 10))

    # -> Subplot para cada junta Posição
    plt.subplot(6, 1, 1)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_1, label='Junta 1', color='red')
    plt.title('Posições das Juntas ao Longo do Tempo (rad)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_2, label='Junta 2', color='orange')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 3)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_3, label='Junta 3', color='green')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 4)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_4, label='Junta 4', color='blue')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_5, label='Junta 5', color='purple')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, config_rads_array_plot_6, label='Junta 6', color='brown')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()



    # ==> Gráficos das Velocidades das juntas
    # Gráficos das Velocidades das juntas
    plt.figure(3, figsize=(12, 10))

    # Subplot para cada junta Velocidade
    plt.subplot(6, 1, 1)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_1, label='Junta 1', color='red')
    plt.title('Velocidade das Juntas ao Longo do Tempo (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_2, label='Junta 2', color='orange')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 3)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_3, label='Junta 3', color='green')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 4)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_4, label='Junta 4', color='blue')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 5)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_5, label='Junta 5', color='purple')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 6)
    plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, vel_config_array_plot_6, label='Junta 6', color='brown')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()



    # ==> Gráficos das Velocidades Cartesianas
    plt.figure(4, figsize=(10, 6))

    # Subplot para Cartesiana Velocidade
    plt.subplot(6, 1, 1)

    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, cartesian_velocities_array_plot_1, label='v_x', color='red')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, cartesian_velocities_array_plot_1, label='v_y', color='red')
    
    plt.title('Velocidade Cartesianas ao Longo do Tempo')
    plt.ylabel('Velocidade (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()

    plt.subplot(6, 1, 2)

    if (opcao == "1" or opcao == "3" or opcao == "5"):
        plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, cartesian_velocities_array_plot_2, label='v_y', color='red')

    elif(opcao == "2" or opcao == "4" or opcao == "6"):
        plt.plot(np.array(range(len(config_rads_array_plot_1))) * iterationTime, cartesian_velocities_array_plot_2, label='v_z', color='red')
    
    plt.ylabel('Velocidade (rad/s)')
    plt.xlabel('Tempo (s)')
    plt.grid(True)
    plt.legend()
    

    plt.show()




"""
    Esta função retorna a Equação escolhida pelo utilizador para dar plot e desenhar a trajetória de referência 
"""
def Compute_Equations(opcao, C, r, r_a, r_b):
    theta = np.linspace(0, 2 * np.pi, 1000)

    if(opcao == "1"):
        pos_1 = C[0] + r * cos(theta)
        pos_2 = C[1] + r * sin(theta) * cos(theta)

    elif(opcao == "2"):
        pos_1 = C[1] + r * cos(theta)
        pos_2 = C[2] + r * sin(theta) * cos(theta)

    elif(opcao == "3"):
        pos_1 = C[0] + r_a * cos(theta)
        pos_2 = C[1] + r_b * sin(theta)

    elif(opcao == "4"):
        pos_1 = C[1] + r_a * cos(theta)
        pos_2 = C[2] + r_b * sin(theta)

    elif(opcao == "5"):
        pos_1 = C[0] + r * cos(theta)
        pos_2 = C[1] + r * sin(theta)

    elif(opcao == "6"):
        pos_1 = C[1] + r * cos(theta)
        pos_2 = C[2] + r * sin(theta)

    return pos_1, pos_2