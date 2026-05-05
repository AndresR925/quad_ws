import rclpy
from rclpy.node import Node
import numpy as np

from quad_msgs.msg import JoyCtrlCmds
from quad_msgs.msg import Geometry
from geometry_msgs.msg import Twist
from quad_ik.quad_ik.InverseKinematics import InverseKinematics
from quad_variables import Body, Leg, Cmds

from std_msgs.msg import String
import threading
from threading import Thread
import logging
import time

class CmdManager_ROS():
    def __init__(self, set_msgs, send_msgs, node_name = 'gait_node'):

        super(CmdManager_ROS, self).__init__()
        # ROS parameters
        self.node = None
        self.node_name = node_name
        # -----   sub1  -----------
        self.sub1 = None
        self.sub1_name = 'hyperdog_joy_ctrl_cmd' #Suscripción a los comandos del control
        self.sub1_interface = JoyCommands  #hyperdog_msgs.msg.JoyCommands
        self.sub1_callback = self._joy_cmd_callback #Llama a la función cada vez que recibe un mensaje
        self.sub1_queueSize = 30 #Cola de 30 mensajes
        # -----   sub2  -----------
        self.sub2 = None
        self.sub2_name = 'vel_cmd' #Comandos de Velocidad
        self.sub2_interface = Twist     #geometry_msgs.msg.Twist(Estándar ROS2)
        self.sub2_callback = self._sub2_callback #Llama a la función cada vez que recibe un mensaje
        self.sub2_queueSize = 10 #Cola de 10 mensajes
        # -----   pub  -----------
        self.pub = None
        self.pub_name = 'quad_geometry'
        self.pub_interface = LegPosition   #hyperdog_msgs.msg.LegPosition
        self.pub_timer_period = 0.001 #Publica cada 1ms(OJO: Puede ser bastante bajo)
        self.pub_timer = None
        self.pub_queueSize = 12 #Tamaño de cola de 12
        self.pub_callback = self._pub_callback

        self.stop = True #flag de control

        # Robot cmds
        self.cmd = set_msgs #Almacena los comandos de movimiento
        self.pub_msgs = send_msgs #Envía los comandos

        #Inicializa el nodo

    def _createNode(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)
        # self.node.get_logger().info('{} node was created!'.format(self.node_name))

    #Suscripción a comandos de control
    def create_sub1(self):
        self.sub1 = self.node.create_subscription(
                        self.sub1_interface,
                        self.sub1_name,
                        self.sub1_callback,
                        self.sub1_queueSize)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.sub1_name))

    #Suscripción a comandos de velocidad de ROS2

    def create_sub2(self):
        self.sub2 = self.node.create_subscription(
                        self.sub2_interface,
                        self.sub2_name,
                        self.sub2_callback,
                        self.sub2_queueSize)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.sub2_name))

    #Creación del publicador

    def create_pub(self):
        self.pub = self.node.create_publisher(
                            self.pub_interface,
                            self.pub_name,
                            self.pub_queueSize
                        )
        self.pub_timer = self.node.create_timer(self.pub_timer_period, self.pub_callback)
        # self.node.get_logger().info('{} subscriber was created!'.format(self.pub_name))


    #Funciones a ejecutar
    #1. Callback de entrada


    def _joy_cmd_callback(self, msg):
        # ------------------------------------------
        self.cmd.mode.start = msg.states[0]
        if self.cmd.mode.start: #Si ha inicializado el robot
            self.cmd.mode.walk = msg.states[1] #Modo caminata normal
            self.cmd.mode.side_walk_mode = msg.states[2] #Modo caminar lado
            self.cmd.mode.gait_type = msg.gait_type #Tipo de gait(trot, walk, run, demo)
            #Establece alturas, desplazamientos lineales, un offset del cuerpo sobre el plano XY.
            # ------------------------------------------
            self.cmd.body.height = msg.pose.position.z
            self.cmd.body.slant[0] = msg.pose.position.x
            self.cmd.body.slant[1] = msg.pose.position.y
            #Orientaciones
            # ------------------------------------------
            self.cmd.body.roll = msg.pose.orientation.x
            self.cmd.body.pitch = msg.pose.orientation.y
            self.cmd.body.yaw = msg.pose.orientation.z
            # ------------------------------------------
            #Ancho del paso
            self.cmd.gait.step_len[0] = msg.gait_step.x
            self.cmd.gait.step_len[1] = msg.gait_step.y
            self.cmd.gait.swing_step_h = msg.gait_step.z

    #Ajusta tiempo de ciclo y tiempo de fase swing según mensajes Twist

    def _sub2_callback(self, msg):
        self.cmd.gait.cycle_time = msg.linear.x
        self.cmd.gait.swing_time = msg.angular.x

    #Publica las posiciones

    def _pub_callback(self):
        msg = Geometry()

        msg.fr.x= self.pub_msgs[0].FR.pose.cur_coord[0]
        msg.fr.y= self.pub_msgs[0].FR.pose.cur_coord[1]
        msg.fr.z= self.pub_msgs[0].FR.pose.cur_coord[2]
        msg.fl.x= self.pub_msgs[0].FL.pose.cur_coord[0]
        msg.fl.y= self.pub_msgs[0].FL.pose.cur_coord[1]
        msg.fl.z= self.pub_msgs[0].FL.pose.cur_coord[2]
        msg.br.x= self.pub_msgs[0].BR.pose.cur_coord[0]
        msg.br.y= self.pub_msgs[0].BR.pose.cur_coord[1]
        msg.br.z= self.pub_msgs[0].BR.pose.cur_coord[2]
        msg.bl.x= self.pub_msgs[0].BL.pose.cur_coord[0]
        msg.bl.y= self.pub_msgs[0].BL.pose.cur_coord[1]
        msg.bl.z= self.pub_msgs[0].BL.pose.cur_coord[2]

        msg.euler_ang.x = np.deg2rad(self.pub_msgs[1].roll)
        msg.euler_ang.y = np.deg2rad(self.pub_msgs[1].pitch)
        msg.euler_ang.z = np.deg2rad(self.pub_msgs[1].yaw)

        self.pub.publish(msg)

        # self.node.get_logger().info('Publishing message')

    #Manejo de hilos

    def get_numOf_threads(self):
        return threading.active_count()

    # Correr el nodo

    def start(self):

        self._createNode()
        self.create_sub1()
        self.create_sub2()
        self.create_pub()
        rclpy.spin(self.node)
        self.node.destroy_node()
        rclpy.shutdown()
        self.stop = False



    def stop_node(self):
        self.stop = True

    def run(self):
        pass