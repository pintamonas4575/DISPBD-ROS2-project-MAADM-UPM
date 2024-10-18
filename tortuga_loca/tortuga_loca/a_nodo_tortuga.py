#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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


class NodoTortuga(Node):
    def __init__(self):
        super().__init__('a_nodo_tortuga')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controla el turtlesim con WASD. Pulsa 'q' para salir.")

    def run(self):
        twist = Twist()
        while rclpy.ok():
            key = get_my_key()

            # Reiniciar las velocidades antes de cada ciclo
            twist.linear.x = 0.0
            twist.angular.z = 0.0

            if key == 'w':
                twist.linear.x = 2.0  # Avanza
            elif key == 's':
                twist.linear.x = -2.0  # Retrocede

            if key == 'a':
                twist.angular.z = 2.0  # Gira a la izquierda
            elif key == 'd':
                twist.angular.z = -2.0  # Gira a la derecha
            elif key == 'q':
                break

            # Publicar el mensaje de velocidad
            self.publisher_.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = NodoTortuga()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
