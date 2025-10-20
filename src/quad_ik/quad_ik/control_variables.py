import numpy as np
#TODO: Cambiar parámetros segun robot real
MIN_LEG_HEIGHT = 80 #Altura mínima
MAX_LEG_HEIGHT = 240 #Altura máxima
MAX_ROLL       = 45 #Maximo Roll
MAX_PITCH      = 45 #Máximo Pitch
MAX_YAW        = 50  #Máximo Yaw

class RobotState():
    start = False #Inicio
    walk  = False #Caminando
    side_move_mode = 0 #Modo lateral
    height = MIN_LEG_HEIGHT #Altura inicial
    eular_ang = [0, 0, 0] #RPY iniciales
    zeroPnts = [[0,0], [0,0], [0,0], [0,0]] #Puntos inicio cada pata
    speed = 0 #Velocidad movimiento

class GaitParameters():
    step_length = [0, 0] #Longitud paso en [x, y], z es constante
    step_height = 0 #Altura del paso
    step_depth = 0 #Desplazamiento vertical de apoyo
    freq       = 0 #Frecuencia del paso
    swing_time = 0 #Duración fase swing
    start_pnt  = [0, 0] #Punto de inicio
    end_pnt    = [0, 0] #Punto final

class LegParameters():
    zeroPnt = [0,0] #Posición de reposo
    cur_pose = [0, 0, 0] #Posición actual
    target_pose = [0,0, 0] #Posición objetivo
    z_err    = 0 #Error en eje z

class LegsParameters():
    FR = LegParameters()
    FL = LegParameters()
    BR = LegParameters()
    BL = LegParameters()

class JointAngles():
    cur_angle    = [0, 0, 0] #Ángulos actuales
    target_angle = [0, 0, 0] #Ángulos objetivo