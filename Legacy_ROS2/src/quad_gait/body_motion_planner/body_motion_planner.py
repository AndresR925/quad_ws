import numpy as np
from nodes.quad_variables import Cmds, Body, Leg
import time
# cmd = Cmds()
# leg = Leg()
# body = Body()

class BodyMotionPlanner():
    def __init__(self , cmd, leg, body, gait_planner):
        self.cmd = cmd
        self.body = body
        self.leg = leg
        self.gait = gait_planner

        self.prev_slant = self.cmd.body.slant #Inclinación del cuerpo previa

        self.__L1 = self.leg.physical._L1 #Longitudes físicas
        self.__L2 = self.leg.physical._L2
        self.__L3 = self.leg.physical._L3

        self.cmd.leg.foot_zero_pnt[:,1] = self.__L1 #Posición q1 = 0 -> No hay desfase en el eje Y.

    def set_init_pose(self):
        #Orientación nula
        self.body.roll = 0
        self.body.pitch = 0
        self.body.yaw = 0
        #Establecer altura sobre el eje z a la altura del robot[80, 240] mm.
        self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height)
        #Asignación de coordenadas iniciales a cada pata
        self.leg.FR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:])
        self.leg.FL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:])
        self.leg.BR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:])
        self.leg.BL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:])
        return True

    def change_height(self):
        #Cambio de altura
        self.leg.FR.pose.cur_coord[2] = self.body.height[:]
        self.leg.FL.pose.cur_coord[2] = self.body.height[:]
        self.leg.BL.pose.cur_coord[2] = self.body.height[:]
        self.leg.BR.pose.cur_coord[2] = self.body.height[:]
        return True

    def run(self):
        while True:
            self.cmd.leg.foot_zero_pnt[:,2] = np.array(self.cmd.body.height) #Actualiza altura según altura actual
            """ uncomment below 2 lines to activate slant from joystick"""
            self.cmd.leg.foot_zero_pnt[:,1] = self.__L1 #Iniciación eje Y
            # self.cmd.leg.foot_zero_pnt[::2,:2] = np.array([0,self.__L1]) + self.cmd.body.slant[:2]
            # self.cmd.leg.foot_zero_pnt[1::2,:2] = np.array([0,self.__L1]) + self.cmd.body.slant[:2]*np.array([1,-1])
            self.body.roll = self.cmd.body.roll #Asignación de ángulos de Euler
            self.body.pitch = self.cmd.body.pitch
            self.body.yaw = self.cmd.body.yaw
            # if np.any(self.cmd.body.slant != self.prev_slant):
            #     self.leg.FR.pose.cur_coord[:2] =

            #Actualizar según: Posición base + Trayectoria generada por el gait planner + Zero Moment Point Stability

            self.leg.FR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[0,:]) + self.gait.FR_traj[:] + self.body.ZMP_handler[0,:]*np.array([0,1,0]) #+ self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.FL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[1,:]) + self.gait.FL_traj[:] + self.body.ZMP_handler[1,:]*np.array([0,1,0]) #+ self.cmd.body.slant[:]*np.array([1,-1,0])
            self.leg.BR.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[2,:]) + self.gait.BR_traj[:] + self.body.ZMP_handler[2,:]*np.array([0,1,0])#+ self.cmd.body.slant[:]*np.array([1,1,0])
            self.leg.BL.pose.cur_coord[:] = np.array(self.cmd.leg.foot_zero_pnt[3,:]) + self.gait.BL_traj[:] + self.body.ZMP_handler[3,:]*np.array([0,1,0])#+ self.cmd.body.slant[:]*np.array([1,-1,0])

            time.sleep(0.0002)