from gait_node import CmdManager_ROS
from quad_variables import Body, Leg, Cmds
from body_motion_planner.body_motion_planner import BodyMotionPlanner
from gait_generator.gait_planner import GaitPlanner
from multiprocessing import Process
import threading
import time
import rclpy

#Creación de instancias globales de las estructuras del robot

leg = Leg()
body = Body()
cmd = Cmds()


# lilnk  objects
# leg.foot_zero_pnt[:,2] = body.height  = cmd.body.height
#Inicialización parámetros por defecto
cmd.gait.cycle_time = 0.8
cmd.gait.swing_time = 0.2
cmd.leg.foot_zero_pnt[0,0] = -10 #FR
cmd.leg.foot_zero_pnt[1,0] = -10 #FL
cmd.leg.foot_zero_pnt[2,0] = -70 #BR
cmd.leg.foot_zero_pnt[3,0] = -70 #BL
cmd.gait.stance_step_h = 0

#Instancias de los módulos

cmd_manager = CmdManager_ROS(set_msgs=cmd, send_msgs=[leg, body]) #Nodo ROS que recibe comandos externos (por ejemplo joystick o teleop) y actualiza cmd.
gait_planner = GaitPlanner(cmd, leg, body) #Planificador
bmp = BodyMotionPlanner(cmd, leg, body, gait_planner) # Mantiene estable el cuerpo durante el movimiento (cinemática + compensación).

gait_planner.len_zmp_wavegait = 50 #Ajusta la longitud del vector de ZMP (Zero Moment Point) para el “wave gait”.Esto define cuántos puntos intermedios se calculan para la trayectoria de equilibrio.Cuanto mayor el valor → más suave el desplazamiento del centro de masa.

def main(args=None): #Inicia y lanza los hilos
    print('starting')

    thread_cmd_manager = threading.Thread(target=cmd_manager.start)
    thread_gait_planner = threading.Thread(target=gait_planner.run)
    thread_bmp = threading.Thread(target=bmp.run)

    # thread_cmd_manager = Process(target=cmd_manager.start)
    # thread_gait_planner = Process(target=gait_planner.run)


    bmp.set_init_pose() #Posición incial

    try:
        thread_cmd_manager.start()
        thread_bmp.start()
        thread_gait_planner.start()

        #Arranque de hilos

        while 1:
            print("tnumber of threads in background: {}".format(threading.active_count()))
            # print("current thread: {}\n".format(threading.current_thread().name))
            # print(cmd.body.height, '----', leg.FR.pose.cur_coord)
            time.sleep(1)
            # print(cmd.mode.start)


    except  KeyboardInterrupt: #Apagado controlado
        cmd_manager.node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()