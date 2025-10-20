import numpy as np
import matplotlib.pyplot as plt
# from time import time
import time
from mpl_toolkits.mplot3d import Axes3D
import math

class GaitPlanner():
    def __init__(self, cmd, leg, body):
        self.cmd = cmd
        self.leg = leg
        self.body = body

        self.gnd_touched = np.ones([4]) #fr,fl,br,bl #Estado de contacto con el suelo(por defecto, True)
        self.sample_time = 0.001 #Tiempo de muestreo en s

        #Inicialización de trayectorias en 0,0,0

        self.FR_traj = np.zeros([3])
        self.FL_traj = np.zeros([3])
        self.BR_traj = np.zeros([3])
        self.BL_traj = np.zeros([3])

        #Historial de trayectoria

        self.fr_traj = []
        self.fl_traj = []
        self.br_traj = []
        self.bl_traj = []

        self.t_zmp_wavegait = 0.5 #duración del ciclo de ZMP para wave gait.
        self.len_zmp_wavegait = 50 #número de muestras para ese ciclo. -> Actualización cada 10ms del ZMP

        #Duración de ciclos

        self.wavegait_cycle_time = 1
        self.trot_gait_cycle_time = 0.5
        self.trot_gait_swing_time = self.cmd.gait.cycle_time/4

    def swing_FR(self, t):
        traj_pnt = np.zeros([3]) #Inicialización de la trayectoria en ceros
        self.leg.FR.gait.swing.time =  self.cmd.gait.swing_time #Tiempo en fase swing
        if self.cmd.mode.walk: #Si el modo es caminar
            self.leg.FR.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2 #[x,y] del punto de finalización
            self.leg.FR.gait.swing.end_pnt[2] = self.leg.FR.pose.cur_coord[2] # altura actual de la pata(el pie no toca el suelo)
            # set start point
            if self.leg.FR.gait.swing.start == False: #Si no ha iniciado la fase swing
                self.leg.FR.gait.stance.start = False #Deshabilitar fase stance
                self.leg.FR.gait.swing.start = True #Habilitar fase swing
                self.leg.FR.gait.swing.start_pnt[:2] = self.leg.FR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] - self.body.ZMP_handler[0,:2] #Punto inicial de la fase swing(Pata relativa al cuerpo)
                self.leg.FR.gait.swing.start_pnt[2] = self.leg.FR.pose.cur_coord[2] #Altura actual
            # Generar trayectoria
            T = self.leg.FR.gait.swing.time #Tiempo de duración

            traj_pnt[:2] = self.leg.FR.gait.swing.start_pnt[:2] + (self.leg.FR.gait.swing.end_pnt[:2] - self.leg.FR.gait.swing.start_pnt[:2])*t/T #[x,y]
            traj_pnt[2] = - np.array(self.cmd.gait.swing_step_h)* np.sin(t/T*np.pi) #[z]
            # end point
            if t >= T - self.sample_time*2: #Si la fase esta a punto de terminar
                traj_pnt[:2] = np.array(self.leg.FR.gait.swing.end_pnt)[:2] #Nos aseguramos que llegue al punto final
                traj_pnt[2] = 0 #La altura debe ser cero(relativa al marco del suelo)
                self.leg.FR.gait.swing.start = False #Finaliza fase swing

        else:
            traj_pnt[:2] = self.leg.FR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] #Pata fija sin movimiento
            traj_pnt[2] = 0 #Mantememos pata en el suelo
        self.FR_traj = np.array(traj_pnt) #Devolvemos array numpy de trayectoria generada

    def stance_FR(self, t):
        traj_pnt = np.zeros([3]) #Inicialización de la trayectoria en ceros
        self.leg.FR.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time #Tiempo en fase stance
        if self.cmd.mode.walk: #Si el modo es caminar
            self.leg.FR.gait.stance.end_pnt[:2] = - np.array(self.cmd.gait.step_len)/2 #retrocede la pata la mitad del step length hacia atrás, porque el cuerpo avanza mientras la pata está apoyada.
            self.leg.FR.gait.stance.end_pnt[2] = self.leg.FR.pose.cur_coord[2] #No modificamos Z
            # set start point
            if self.leg.FR.gait.stance.start == False: #Si la fase stance no esta activa
                self.leg.FR.gait.stance.start = True #Activar fase stance
                self.leg.FR.gait.stance.start_pnt[:2] = self.leg.FR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] - self.body.ZMP_handler[0,:2] #Punto inicial de la fase stance(Pata relativa al cuerpo)
                self.leg.FR.gait.stance.start_pnt[2] = self.leg.FR.pose.cur_coord[2] #Mantener altura Z
            # make trajectory

            T = self.leg.FR.gait.stance.time #Tiempo de duración
            traj_pnt[:2] = self.leg.FR.gait.stance.start_pnt[:2] + (self.leg.FR.gait.stance.end_pnt[:2] - self.leg.FR.gait.stance.start_pnt[:2])*t/T #[x, y]
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi) #[Z] -> la pata no sube mucho, solo hace un ligero “rebote” sobre el suelo.
            # end point
            if t >= T - self.sample_time*2: #Si ya va a finalizar la fase stance
                traj_pnt[:2] = self.leg.FR.gait.stance.end_pnt[:2] #Nos aseguramos que llegue al punto final
                traj_pnt[2] = 0 #La altura debe ser cero(relativa al marco del suelo)
                self.leg.FR.gait.stance.start = False #Finaliza fase stance
        else:
            traj_pnt[:2] = self.leg.FR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[0,:2] #Pata fija sin movimiento
            traj_pnt[2] = 0 #Mantememos pata en el suelo
        self.FR_traj = np.array(traj_pnt) #Devolvemos array numpy de trayectoria generada

      #DE IGUAL FORMA EN LAS DEMAS EXTREMIDADES ASÍ:

    def swing_FL(self, t):
        traj_pnt = np.zeros([3])
        self.leg.FL.gait.swing.time =  np.array(self.cmd.gait.swing_time)
        if self.cmd.mode.walk:
            self.leg.FL.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2 * np.array([1,-1])
            self.leg.FL.gait.swing.end_pnt[2] = self.leg.FL.pose.cur_coord[2]

            if self.leg.FL.gait.swing.start == False:
                self.leg.FL.gait.stance.start = False
                self.leg.FL.gait.swing.start = True
                self.leg.FL.gait.swing.start_pnt[:2] = self.leg.FL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2] - self.body.ZMP_handler[1,:2]
                self.leg.FL.gait.swing.start_pnt[2] = self.leg.FL.pose.cur_coord[2]
            # make trajectory
            T = self.leg.FL.gait.swing.time
            traj_pnt[:2] = self.leg.FL.gait.swing.start_pnt[:2] + (self.leg.FL.gait.swing.end_pnt[:2] - self.leg.FL.gait.swing.start_pnt[:2])*t/T
            traj_pnt[2] = - self.cmd.gait.swing_step_h* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.FL.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.FL.gait.swing.start = False

        else:
            traj_pnt[:2] = self.leg.FL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2]
            traj_pnt[2] = 0
        self.FL_traj = np.array(traj_pnt)


    def stance_FL(self, t):
        traj_pnt = np.zeros([3])
        self.leg.FL.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk:
            self.leg.FL.gait.stance.end_pnt[:2] = - self.cmd.gait.step_len/2 * np.array([1,-1])
            self.leg.FL.gait.stance.end_pnt[2] = self.leg.FL.pose.cur_coord[2]
            # set start point
            if self.leg.FL.gait.stance.start == False:
                self.leg.FL.gait.stance.start = True
                self.leg.FL.gait.stance.start_pnt[:2] = self.leg.FL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2] - self.body.ZMP_handler[1,:2]
                self.leg.FL.gait.stance.start_pnt[2] = self.leg.FL.pose.cur_coord[2]

            # make trajectory
            T = self.leg.FL.gait.stance.time
            traj_pnt[:2] = self.leg.FL.gait.stance.start_pnt[:2] + (self.leg.FL.gait.stance.end_pnt[:2] - self.leg.FL.gait.stance.start_pnt[:2])*t/T
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.FL.gait.stance.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.FL.gait.stance.start = False
        else:
            traj_pnt[:2] = self.leg.FL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[1,:2]
            traj_pnt[2] = 0
        self.FL_traj = np.array(traj_pnt) * np.array([1,1,1])

    def swing_BR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.BR.gait.swing.time =  self.cmd.gait.swing_time
        if self.cmd.mode.walk:
            if (self.cmd.mode.side_walk_mode == 0):
                self.leg.BR.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2
            else:
                self.leg.BR.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len)/2 * np.array([1,-1]) #ojo puede ser [-1,1]
            self.leg.BR.gait.swing.end_pnt[2] = self.leg.BR.pose.cur_coord[2]
            # set start point
            if self.leg.BR.gait.swing.start == False:
                self.leg.BR.gait.stance.start = False
                self.leg.BR.gait.swing.start = True
                self.leg.BR.gait.swing.start_pnt[:2] = self.leg.BR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]  - self.body.ZMP_handler[2,:2]
                self.leg.BR.gait.swing.start_pnt[2] = self.leg.BR.pose.cur_coord[2]
            # make trajectory
            T = self.leg.BR.gait.swing.time
            traj_pnt[:2] = self.leg.BR.gait.swing.start_pnt[:2] + (self.leg.BR.gait.swing.end_pnt[:2] - self.leg.BR.gait.swing.start_pnt[:2])*t/T
            traj_pnt[2] = - np.array(self.cmd.gait.swing_step_h)* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.BR.gait.swing.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.BR.gait.swing.start = False

        else:
            traj_pnt[:2] = self.leg.BR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]
            traj_pnt[2] = 0
        self.BR_traj = np.array(traj_pnt)


    def stance_BR(self, t):
        traj_pnt = np.zeros([3])
        self.leg.BR.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk:
            if (self.cmd.mode.side_walk_mode == 0):
                self.leg.BR.gait.stance.end_pnt[:2] = - np.array(self.cmd.gait.step_len)/2
            else:
                self.leg.BR.gait.stance.end_pnt[:2] = - np.array(self.cmd.gait.step_len)/2 * np.array([1,-1]) #ojo puede ser [-1,1]
            self.leg.BR.gait.stance.end_pnt[2] = self.leg.BR.pose.cur_coord[2]
            # set start point
            if self.leg.BR.gait.stance.start == False:
                self.leg.BR.gait.stance.start = True
                self.leg.BR.gait.stance.start_pnt[:2] = self.leg.BR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]  - self.body.ZMP_handler[2,:2]
                self.leg.BR.gait.stance.start_pnt[2] = self.leg.BR.pose.cur_coord[2]
            # make trajectory
            T = self.leg.BR.gait.stance.time
            traj_pnt[:2] = self.leg.BR.gait.stance.start_pnt[:2] + (self.leg.BR.gait.stance.end_pnt[:2] - self.leg.BR.gait.stance.start_pnt[:2])*t/T
            traj_pnt[2] = np.array(self.cmd.gait.stance_step_h)* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = np.array(self.leg.BR.gait.stance.end_pnt)[:2]
                traj_pnt[2] = 0
                self.leg.BR.gait.stance.start = False
        else:
            traj_pnt[:2] = self.leg.BR.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[2,:2]
            traj_pnt[2] = 0
        self.BR_traj = np.array(traj_pnt)


    def swing_BL(self, t):
        traj_pnt = np.zeros([3])
        self.leg.BL.gait.swing.time =  self.cmd.gait.swing_time
        if self.cmd.mode.walk:
            if (self.cmd.mode.side_walk_mode == 0):
                self.leg.BL.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len/2) * np.array([1,-1]) #ojo puede ser [-1,-1]
            else:
                self.leg.BL.gait.swing.end_pnt[:2] = np.array(self.cmd.gait.step_len/2)
            self.leg.BL.gait.swing.end_pnt[2] = self.leg.BL.pose.cur_coord[2]

            if self.leg.BL.gait.swing.start == False:
                self.leg.BL.gait.stance.start = False
                self.leg.BL.gait.swing.start = True
                self.leg.BL.gait.swing.start_pnt[:2] = self.leg.BL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2] - self.body.ZMP_handler[3,:2]
                self.leg.BL.gait.swing.start_pnt[2] = self.leg.BL.pose.cur_coord[2]
            # make trajectory
            T = self.leg.BL.gait.swing.time
            traj_pnt[:2] = self.leg.BL.gait.swing.start_pnt[:2] + (self.leg.BL.gait.swing.end_pnt[:2] - self.leg.BL.gait.swing.start_pnt[:2])*t/T
            traj_pnt[2] = - self.cmd.gait.swing_step_h* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = self.leg.BL.gait.swing.end_pnt[:2]
                traj_pnt[2] = 0
                self.leg.BL.gait.swing.start = False

        else:
            traj_pnt[:2] = self.leg.BL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2]
            traj_pnt[2] = 0
        self.BL_traj = np.array(traj_pnt) * np.array([1,1,1])


    def stance_BL(self, t):
        traj_pnt = np.zeros([3])
        self.leg.BL.gait.stance.time = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
        if self.cmd.mode.walk:
            if (self.cmd.mode.side_walk_mode == 0):
                self.leg.BL.gait.stance.end_pnt[:2] = - self.cmd.gait.step_len/2 * np.array([1,-1]) #ojo puede ser [-1,-1]
            else:
                self.leg.BL.gait.stance.end_pnt[:2] = - self.cmd.gait.step_len/2
            self.leg.BL.gait.stance.end_pnt[2] = self.leg.BL.pose.cur_coord[2]
            # set start point
            if self.leg.BL.gait.stance.start == False:
                self.leg.BL.gait.stance.start = True
                self.leg.BL.gait.stance.start_pnt[:2] = self.leg.BL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2]  - self.body.ZMP_handler[3,:2]
                self.leg.BL.gait.stance.start_pnt[2] = self.leg.BL.pose.cur_coord[2]

            # make trajectory
            T = self.leg.BL.gait.stance.time
            traj_pnt[:2] = self.leg.BL.gait.stance.start_pnt[:2] + (self.leg.BL.gait.stance.end_pnt[:2] - self.leg.BL.gait.stance.start_pnt[:2])*t/T
            traj_pnt[2] = self.cmd.gait.stance_step_h* np.sin(t/T*np.pi)
            # end point
            if t >= T - self.sample_time*2:
                traj_pnt[:2] = self.leg.BL.gait.stance.end_pnt[:2]
                traj_pnt[2] = 0
                self.leg.BL.gait.stance.start = False
        else:
            traj_pnt[:2] = self.leg.BL.pose.cur_coord[:2] - self.cmd.leg.foot_zero_pnt[3,:2]
            traj_pnt[2] = 0
        self.BL_traj = np.array(traj_pnt) * np.array([1,1,1])

    #Debug de trayectorias

    def __plot_debug(self, period):
        traj_fr = np.array(self.fr_traj)
        t_fr = np.linspace(0,period, len(traj_fr))
        traj_fl = np.array(self.fl_traj)
        t_fl = np.linspace(0,period, len(traj_fl))
        traj_br = np.array(self.br_traj)
        t_br = np.linspace(0,period, len(traj_br))
        traj_bl = np.array(self.bl_traj)
        t_bl = np.linspace(0,period, len(traj_bl))

        fig, axs = plt.subplots(2,2, dpi = 250)
        axs[0,1].plot(t_fr, traj_fr[:,0], label = 'FRx')
        axs[0,1].plot(t_fr, traj_fr[:,1], label = 'FRy')
        axs[0,1].plot(t_fr, traj_fr[:,2], label = 'FRz')
        axs[0,1].set_title('FR')

        axs[0,0].plot(t_fl, traj_fl[:,0], label = 'FLx')
        axs[0,0].plot(t_fl, traj_fl[:,1], label = 'FLy')
        axs[0,0].plot(t_fl, traj_fl[:,2], label = 'FLz')
        axs[0,0].set_title('FL')

        axs[1,1].plot(t_br, traj_br[:,0], label = 'BRx')
        axs[1,1].plot(t_br, traj_br[:,1], label = 'BRy')
        axs[1,1].plot(t_br, traj_br[:,2], label = 'BRz')
        axs[1,1].set_title('BR')

        axs[1,0].plot(t_bl, traj_bl[:,0], label = 'BLx')
        axs[1,0].plot(t_bl, traj_bl[:,1], label = 'BLy')
        axs[1,0].plot(t_bl, traj_bl[:,2], label = 'BLz')
        axs[1,0].set_title('BL')
        plt.legend()

    #debug trayectoria trotar

    def trot_gait_debug(self, period):

        #Inicialización arrays vacios de trayectoria
        self.fr_traj = []
        self.fl_traj = []
        self.br_traj = []
        self.bl_traj = []
        #Inicialización de tiempos
        t = time.time()
        dt = time.time() - t
        p = time.time()
        dp = time.time() - p
        i = 0
        # run gait for 10 sec
        while dp <= period:
            if dt <= self.cmd.gait.cycle_time: #Comprueba si el intervalo de tiempo es menor a todo el tiempo que dura el ciclo
                if dt >= self.sample_time*i: #Calcular nuevas posiciones solo si ya ha pasado el tiempo de muestreo
                    i += 1 #Incrementar iterador
                    if dt <= self.cmd.gait.swing_time: #Si estamos en Swing
                        #Dos patas en swing, otras dos en stance
                        coord_fr = np.array(self.swing_FR(dt))
                        coord_fl = np.array(self.stance_FL(dt))
                        coord_br = np.array(self.stance_BR(dt))
                        coord_bl = np.array(self.swing_BL(dt))
                        #Añadimos coordendas resultantes
                        self.fr_traj.append(coord_fr)
                        self.fl_traj.append(coord_fl)
                        self.br_traj.append(coord_br)
                        self.bl_traj.append(coord_bl)
                    elif dt > self.cmd.gait.swing_time and dt < self.cmd.gait.cycle_time - self.cmd.gait.swing_time: #Si estamos en stance
                        #Aplicamos stance en todas las patas pero con delay de tiempo
                        coord_fr = np.array(self.stance_FR(dt - self.cmd.gait.swing_time))
                        coord_fl = np.array(self.stance_FL(dt))
                        coord_br = np.array(self.stance_BR(dt))
                        coord_bl = np.array(self.stance_BL(dt - self.cmd.gait.swing_time))
                        #Añadimos coordendas resultantes
                        self.fr_traj.append(coord_fr)
                        self.fl_traj.append(coord_fl)
                        self.br_traj.append(coord_br)
                        self.bl_traj.append(coord_bl)
                    else: #Inversión de ciclos
                        stance_t = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
                        coord_fr = np.array(self.stance_FR(dt - self.cmd.gait.swing_time))
                        coord_fl = np.array(self.swing_FL(dt - stance_t))
                        coord_br = np.array(self.swing_BR(dt - stance_t))
                        coord_bl = np.array(self.stance_BL(dt - self.cmd.gait.swing_time))
                        self.fr_traj.append(coord_fr)
                        self.fl_traj.append(coord_fl)
                        self.br_traj.append(coord_br)
                        self.bl_traj.append(coord_bl)
            else: #Si ya pasamos el ciclo, reiniciamos
                t = time.time()
                i = 0
            dt = time.time() - t
            dp = time.time() - p
        self.__plot_debug(period)

    def run_trot(self):

        #Inicialización de contadores
        t = time.time()
        dt = time.time() - t
        i = 0
        while self.cmd.mode.walk: #Si esta en modo caminando
            if dt <= self.cmd.gait.cycle_time: #Si no ha acabado el tiempo de ciclo de marcha
                if dt >= self.sample_time*i: #Si ha pasado el tiempo de muestreo
                    i += 1
                    # FR,BL - swing |   FL,BR - stance
                    if dt <= self.cmd.gait.swing_time:
                        self.swing_FR(dt)
                        self.stance_FL(dt)
                        self.stance_BR(dt)
                        self.swing_BL(dt)
                    # All - stance
                    elif dt > self.cmd.gait.swing_time and dt < self.cmd.gait.cycle_time - self.cmd.gait.swing_time:
                        self.stance_FR(dt - self.cmd.gait.swing_time)
                        self.stance_FL(dt)
                        self.stance_BR(dt)
                        self.stance_BL(dt - self.cmd.gait.swing_time)
                    # FR,BL - stance |   FL,BR - swing
                    else:
                        stance_t = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
                        self.stance_FR(dt - self.cmd.gait.swing_time)
                        self.swing_FL(dt - stance_t)
                        self.swing_BR(dt - stance_t)
                        self.stance_BL(dt - self.cmd.gait.swing_time)
            else:
                # Reset de ciclo
                i = 0
                t = time.time()
            # print(self.leg.FR.gait.traj_pnt[:]) #debug
            dt = time.time() - t
            time.sleep(0.0002)

    #Segundo modo: Transición Stance más rápida

    def run_trot2(self):
        t = time.time()
        dt = time.time() - t
        i = 0
        while self.cmd.mode.walk:
            if dt <= self.cmd.gait.cycle_time:
                if dt >= self.sample_time*i:
                    i += 1
                    # FR,BL - swing |   FL,BR - stance
                    if dt <= self.cmd.gait.swing_time:
                        self.swing_FR(dt)
                        self.stance_FL(dt)
                        self.stance_BR(dt)
                        self.swing_BL(dt)
                    # All - stance
                    elif dt > self.cmd.gait.swing_time and dt < (self.cmd.gait.cycle_time - self.cmd.gait.swing_time)/2:
                        self.stance_FR(dt - self.cmd.gait.swing_time)
                        self.stance_FL(dt)
                        self.stance_BR(dt)
                        self.stance_BL(dt - self.cmd.gait.swing_time)
                    # FR,BL - stance |   FL,BR - swing
                    else:
                        stance_t = self.cmd.gait.cycle_time - self.cmd.gait.swing_time
                        self.stance_FR(dt - self.cmd.gait.swing_time)
                        self.swing_FL(dt - stance_t)
                        self.swing_BR(dt - stance_t)
                        self.stance_BL(dt - self.cmd.gait.swing_time)
            else:
                # cycle reset
                i = 0
                t = time.time()
            # print(self.leg.FR.gait.traj_pnt[:]) #debug
            dt = time.time() - t
            time.sleep(0.0002)

    #Gait en oleada
    def run_waveGait(self):
        stance_zone_count = np.zeros([4]) #Vector de 4 elementos (FR, FL, BR, BL) usado como contadores/offsets de zona para las fases stance.
        #Inicialización de tiempos
        t = time.time()
        dt = time.time() - t
        t_zmp = self.t_zmp_wavegait #Tiempo de manipulación del Zero Moment Point
        zmp_len = self.len_zmp_wavegait #Longitud del Zero Moment Point
        i = 0
        self.body.ZMP_handler[:,:] = 0 #Reset del ZMP

        while self.cmd.mode.walk: #Mientras el robot este caminando
            zone_time = self.cmd.gait.cycle_time/4 #División en cuatro ventanas de tiempo
            if dt <= self.cmd.gait.cycle_time + 2*t_zmp: #Comprueba que el tiempo este dentro del tiempo del ciclo de marcha mas un offset por ZMP.
                if dt >= self.sample_time*i: #Si ha pasado el tiempo de muestreo
                    i += 1
                    # Desplazar ZMP a la izquierda.
                    if dt <= t_zmp/2:
                        self.body.ZMP_handler[::2,1] = dt*zmp_len/(t_zmp/2)
                        self.body.ZMP_handler[1::2,1] = -dt*zmp_len/(t_zmp/2)

                    # BR - swing |   other - stance
                    elif dt > t_zmp/2 and dt <= zone_time + t_zmp/2:
                        self.swing_BR(dt - t_zmp/2)
                        self.stance_FR(stance_zone_count[0]*zone_time + dt - t_zmp/2)
                        self.stance_BL(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_FL(dt- t_zmp/2)
                        stance_zone_count[2] = 0
                    # FR - swing | other stance
                    elif dt > zone_time + t_zmp/2 and dt <= 2*zone_time + t_zmp/2:
                        self.stance_BR(dt-zone_time - t_zmp/2)
                        self.swing_FR(dt-zone_time - t_zmp/2)
                        self.stance_BL(stance_zone_count[3]*zone_time + dt - t_zmp/2)
                        self.stance_FL(dt - t_zmp/2)
                        stance_zone_count[0] = 0

                    # ZMP a la derecha
                    elif dt > 2*zone_time + t_zmp/2 and dt <= 2*zone_time + 3*t_zmp/2:
                        self.body.ZMP_handler[::2,1] = zmp_len-(dt-2*zone_time - t_zmp/2)*2*zmp_len/t_zmp
                        self.body.ZMP_handler[1::2,1] = -zmp_len+ (dt-2*zone_time - t_zmp/2)*2*zmp_len/t_zmp

                    # BL - swing | other - stance
                    elif dt > 2*zone_time + 3*t_zmp/2 and dt <= 3*zone_time + 3*t_zmp/2:
                        self.stance_BR(dt-zone_time - 3/2*t_zmp)
                        self.stance_FR(dt-2*zone_time - 3/2*t_zmp)
                        self.swing_BL(dt-2*zone_time - 3/2*t_zmp)
                        self.stance_FL(dt - 3/2*t_zmp)
                        stance_zone_count[3] = 0
                    # FL - swing | other - stance
                    elif dt > 3*zone_time + 3/2*t_zmp and dt <= 4*zone_time + 3/2*t_zmp:
                        self.stance_BR(dt-zone_time - 3/2*t_zmp)
                        self.stance_FR(dt-2*zone_time - 3/2*t_zmp)
                        self.stance_BL(dt-3*zone_time - 3/2*t_zmp)
                        self.swing_FL(dt-3*zone_time - 3/2*t_zmp)
                        stance_zone_count[1] = 0

                    #Recuperación del ZMP a la posición original

                    else:
                        self.body.ZMP_handler[::2,1] =  -zmp_len + (dt-4*zone_time - 3/2*t_zmp)*zmp_len/(t_zmp/2)
                        self.body.ZMP_handler[1::2,1] = zmp_len -(dt-4*zone_time- 3/2*t_zmp)*zmp_len/(t_zmp/2)

            else:
                #Si ya acabó el ciclo de marcha:
                if np.any(self.cmd.gait.step_len[:2] != 0):
                    #Números para alinear las fases para el próximo ciclo
                    stance_zone_count[0] = 2    # FR
                    stance_zone_count[1] = 0    # FL
                    stance_zone_count[2] = 3    # BR
                    stance_zone_count[3] = 1    # BL
                else:
                    for i in range (4):
                        stance_zone_count[i] = 0 #Resetear a cero si no hay movimiento
                # Reset de ciclo
                self.body.ZMP_handler[:,:] = 0
                i = 0
                t = time.time()

            dt = time.time() - t
            time.sleep(0.0001)

    def give_hand(self):

        #Preparación del ZMP
        self.body.ZMP_handler[::2,1] = self.len_zmp_wavegait
        self.body.ZMP_handler[1::2,1] = -self.len_zmp_wavegait

        #Bucle principal de la función dar la pata

        while self.cmd.mode.gait_type == 0:
            self.FR_traj[0] = self.cmd.gait.step_len[0] #Trayectoria simple, avance en X
            self.FR_traj[2] = self.cmd.gait.step_len[1] #Subir en Z lo que esta desfasado en Y

    #-----BUCLE PRINCIPAL-----#

    def run(self):
        while True:
            if self.cmd.mode.walk: #Si el robot esta en modo caminata
                if self.cmd.mode.gait_type == 1: #Trotar
                    self.cmd.gait.cycle_time = 0.8
                    self.cmd.gait.swing_time = 0.5* self.cmd.gait.cycle_time
                    self.body.ZMP_handler[:,:] = 0
                    self.run_trot()
                elif self.cmd.mode.gait_type == 2: #Caminar
                    self.cmd.gait.cycle_time = 1.5
                    self.cmd.gait.swing_time = 0.2 * self.cmd.gait.cycle_time
                    self.run_waveGait()
                elif self.cmd.mode.gait_type == 3: #Trotar mas rápido
                    self.cmd.gait.cycle_time = 0.8
                    self.cmd.gait.swing_time = 0.2
                    self.body.ZMP_handler[:,:] = 0
                    self.run_trot()
                elif self.cmd.mode.gait_type == 0: #Dar la pata
                    self.give_hand()

            else: #Si no, dejar la altura en cero y reset ZMP.
                self.FR_traj[2] = 0
                self.FL_traj[2] = 0
                self.BR_traj[2] = 0
                self.BL_traj[2] = 0
                self.body.ZMP_handler[:,:] = 0