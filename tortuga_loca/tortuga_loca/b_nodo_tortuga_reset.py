#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty  # Importar el servicio que se usará para resetear
import sys, termios, tty


def get_my_key():
    # Guardar las configuraciones originales del terminal
    original_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        # Restaurar las configuraciones del terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
    return key


class NodoTortugaReset(Node):
    def __init__(self):
        super().__init__('b_nodo_tortuga_reset')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controla el turtlesim con WASD. Pulsa 'R' para resetear la tortuga y 'q' para salir.")

        # Crear un cliente para el servicio de reset
        self.client_reset = self.create_client(Empty, '/reset')
        while not self.client_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio /reset...')

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = get_my_key()

            if key == 'w':
                twist.linear.x = 2.0
                twist.angular.z = 0.0
            elif key == 's':
                twist.linear.x = -2.0
                twist.angular.z = 0.0
            elif key == 'a':
                twist.linear.x = 0.0
                twist.angular.z = 2.0
            elif key == 'd':
                twist.linear.x = 0.0
                twist.angular.z = -2.0
            elif key == 'r':  # Si se pulsa la tecla 'R', se hace el reset
                self.reset_turtle()
                continue  # Saltar el resto del ciclo para que no publique velocidad
            elif key == 'q':
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.publisher_.publish(twist)

        # Detener el turtle cuando se cierra el programa
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def reset_turtle(self):
        # Llamada al servicio para resetear la posición de la tortuga
        req = Empty.Request()
        self.client_reset.call_async(req)
        self.get_logger().info('Posición de la tortuga reiniciada.')


def main(args=None):
    rclpy.init(args=args)
    node = NodoTortugaReset()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
