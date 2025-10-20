import numpy as np


class Cmds():
    class _mode():
        start = False #Robot activo
        walk = False #Robot caminando
        side_walk_mode = 0 #Modo caminata lateral
        gait_type = 0 #Tipo de caminata(trot, walk, run, demo)
    # -----------------
    class _body():
        height = 80 #Altura del cuerpo respecto al suelo
        roll = 0
        pitch = 0
        yaw = 0
        slant = np.zeros([3]) #Offset a cero

    # -----------------
    class _leg():
        foot_zero_pnt = np.zeros([4,3]) # [FR,FL,BR,BL][x,y] #Posición neutra en x,y
    # ------------------
    class _gait():
        step_len = np.zeros([2]) # [len_x,len_y] #Longitud de paso
        swing_step_h = 0 #Altura swing
        stance_step_h = 0 #Altura fase de apoyo
        cycle_time = 0 #Tiempo del ciclo
        swing_time = 0 #Tiempo swing

    mode = _mode()
    body = _body()
    leg = _leg()
    gait = _gait()


#============================================================================

#Clase física

class Body():
    def __init__(self):
        self.height = None #Altura
        self.centerOfMass = np.zeros([3]) #Centro de masa
        self.physical = self._physiacal_params() #Parámetros físicos
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.ZMP_handler = np.zeros([4,3]) #Matriz ZMP
    class _physiacal_params(): #Parámetris físicos(OJO: CAMBIAR ESTO)
        _length = 300
        _width = 172
        _min_height = 80
        _max_height = 240



#============================================================================
#Clase parámetros de pierna
class _LegParams:
    def __init__(self):
        self.pose = self.leg_pose() #Posición(x,y,z)
        self.gait = self.gait_params() #Parámetros de caminata
    class gait_params:
        def __init__(self):
            self.cycle_time = None #Tiempo de ciclo
            self.step_len = np.zeros([3]) #Longitud de paso
            self.stance = self.params() #Fase de stance
            self.swing = self.params() #Fase de swing
            self.traj_pnt = np.zeros([3]) # [x,y,z] #Trayectoria
        class params:
            def __init__(self):
                self.start = False #Inicio
                self.time = None #Final
                self.start_pnt = np.zeros([3]) #Punto inicial
                self.end_pnt = np.zeros([3]) #Punto final


    class leg_pose:
        def __init__(self):
            self.zero_pnt = np.zeros([3]) #Punto neutro
            self.cur_coord = np.zeros([3]) #Coordenada actual
            self.origin_2_endEfector_dist = np.sqrt(self.cur_coord[0]**2 + self.cur_coord[1]**2 + self.cur_coord[2]**2) #Distancia global desde el origen del frame hasta la base de la pata

 # public
class Leg: #Clase general
    def __init__(self):
        self.FR = _LegParams()
        self.FL = _LegParams()
        self.BR = _LegParams()
        self.BL = _LegParams()
        self.physical = self._physical_params()
    class _physical_params(): #OJO CAMBIAR ESTO POR LAS MEDIDAS NUESTRAS
        _L1 = 104 # mm
        _L2 = 150 # mm
        _L3 = 150 # mm



#============================================================================