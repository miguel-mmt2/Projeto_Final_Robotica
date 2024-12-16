#!/usr/bin/env python3

# Importar as Bibliotecas necessárias:
import os

import numpy as np
from numpy import sin
from numpy import cos

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
        cartesian_velocities = np.array([   -r*sin(alpha),
                                            r*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0,
                                            0])
    elif(opcao == "6"): # Equação da Circunferência
        cartesian_velocities = np.array([   0,
                                            -r*sin(alpha),
                                            r*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0])
    return cartesian_velocities



"""
    Esta função retorna as respetivas velocidades cartesianas com base na Equação
"""
def Compute_cartesian_velocity_ideal(opcao, r, r_a, r_b, alpha, alpha_velocity):
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
        cartesian_velocities = np.array([   -r*sin(alpha),
                                            r*cos(alpha)*alpha_velocity,
                                            0,
                                            0,
                                            0,
                                            0])
    elif(opcao == "6"): # Equação da Circunferência
        cartesian_velocities = np.array([   0,
                                            -r*sin(alpha),
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

