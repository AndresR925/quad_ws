import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray
from quad_msgs.msg import LegPosition
from InverseKinematics import InverseKinematics

class InvKin_Node(Node):
    def __init__(self):
        super().__init__('IK_node')
        self.IK =  InverseKinematics() #Clase IK
        self.joint_angs = Float32MultiArray() #Ángulos actuales
        self.prev_joint_angs = None #Ángulos anteriores
        self.sub_ = self.create_subscription(LegPosition, 'quad_msgs', self.sub_callback, 30) #Suscribirse al tópico /LegPosition
        self.pub2STM = self.create_publisher(Float32MultiArray, 'quad_jointController/commands', 30) #Publicar al tópico quad_jointController/commands
        timer_period = 0.02 #50Hz
        # self.timerPub = self.create_timer(timer_period, callback =self.pub_callback1 )
        self.timerPub = self.create_timer(timer_period, self.pub_callback) #Timer para publicar


    def sub_callback(self, msg):
            #Recupera posiciones de las patas y orientación del cuerpo
            eulerAng = np.array([msg.euler_ang.x, msg.euler_ang.y, msg.euler_ang.z])
            fr_coord = np.array([msg.fr.x, msg.fr.y, msg.fr.z])
            fl_coord = np.array([msg.fl.x, msg.fl.y, msg.fl.z])
            br_coord = np.array([msg.br.x, msg.br.y, msg.br.z])
            bl_coord = np.array([msg.bl.x, msg.bl.y, msg.bl.z])

            #Cálcula los ángulos de las articulaciones usando el método get_angles de la clase IK ya implementada

            ang_FR = self.IK.get_FR_joint_angles(fr_coord, eulerAng)
            ang_FL = self.IK.get_FL_joint_angles(fl_coord, eulerAng)
            ang_BR = self.IK.get_BR_joint_angles(br_coord, eulerAng)
            ang_BL = self.IK.get_BL_joint_angles(bl_coord, eulerAng)
            # self.get_logger().info('singularity: {}!'.format(self.IK.singularity))
            if not np.any(self.IK.singularity) \
                and np.any(ang_FR != None) and np.any(ang_FL != None) and np.any(ang_BR != None) and np.any(ang_BL != None): #Verificación de singularidad y arrays válidos
                for i in range (3):
                    #Conversión a grados
                    ang_FR[i] = np.rad2deg(ang_FR[i])
                    ang_FL[i] = np.rad2deg(ang_FL[i])
                    ang_BR[i] = np.rad2deg(ang_BR[i])
                    ang_BL[i] = np.rad2deg(ang_BL[i])
                #Preparación de los datos
                # TODO: Verificar relación entre q2 y q3 en el prototipo físico.
                # En HyperDog, el tercer servo recibe q2+q3 por diseño mecánico.
                # En nuestro modelo, pendiente confirmar si aplica la misma suma.
                self.joint_angs.data = [
                                    ang_FR[0], ang_FR[1], ang_FR[1]+ang_FR[2],
                                    ang_FL[0], ang_FL[1], ang_FL[1]+ang_FL[2],
                                    ang_BR[0], ang_BR[1], ang_BR[1]+ang_BR[2],
                                    ang_BL[0], ang_BL[1], ang_BL[1]+ang_BL[2]
                                    ]
                self.prev_joint_angs = self.joint_angs.data #Actualización de ángulos
                # self.pub2STM.publish(self.joint_angs)
            elif not self.prev_joint_angs == None:
                self.joint_angs.data = self.prev_joint_angs #No publicar en caso de falla o singularidad, actualizar de forma segura
    def pub_callback(self):
        if np.any(self.joint_angs.data) != None: #Verifica que existan datos válidos
            pass
            self.pub2STM.publish(self.joint_angs)  #Publicar en el tópico quad_jointController/commands


def main(args=None):
    rclpy.init(args=args)
    inv_kin = InvKin_Node()
    rclpy.spin(inv_kin)
    inv_kin.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()