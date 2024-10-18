import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import sys, termios, tty
from turtlesim.srv import TeleportAbsolute


def get_my_key():
    """
    Lee una tecla de manera no bloqueante en la terminal.
    """
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key


class NodoTortugaGiros(Node):
    def __init__(self):
        super().__init__('c_nodo_tortuga_giros')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info("Controla el turtlesim con WASD. Pulsa 'R' para resetear la tortuga y 'q' para salir.")

        # Crear un cliente para el servicio de reset
        self.client_reset = self.create_client(Empty, '/reset')
        while not self.client_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio /reset...')

    def run(self):
        twist = Twist()
        linear_speed = 0.0
        angular_speed = 0.0

        while rclpy.ok():
            key = get_my_key()

            # Mover hacia adelante o hacia atrás
            if key == 'w':
                linear_speed = 2.0  # Avanza
            elif key == 's':
                linear_speed = -2.0  # Retrocede
            elif key == 'x':
                linear_speed = 0.0  # Detiene el movimiento hacia adelante o atrás

            # Girar a la izquierda o derecha
            if key == 'a':
                angular_speed = 2.0  # Gira a la izquierda
            elif key == 'd':
                angular_speed = -2.0  # Gira a la derecha
            elif key == 'z':
                angular_speed = 0.0  # Detiene el giro

            if key == 'r': # Reset
                self.reset_turtle()
                continue

            if key == 'q': # Salir
                break

            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.publisher_.publish(twist)

        # Detener el turtle cuando se cierra el programa
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

    def reset_turtle(self):
        req = Empty.Request()
        self.client_reset.call_async(req)
        self.get_logger().info('Posición de la tortuga reiniciada.')

        """twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0  # 0 radianes
        #twist.angular.z = 1.57  # Aproximadamente π/2 radianes
        #twist.angular.z = 3.14  # Aproximadamente π radianes
        #twist.angular.z = 4.71  # Aproximadamente 3π/2 radianes

        self.publisher_.publish(twist)"""


def main(args=None):
    rclpy.init(args=args)
    node = NodoTortugaGiros()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
