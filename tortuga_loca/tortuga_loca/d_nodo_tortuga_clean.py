import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
import sys, termios, tty
from turtlesim.srv import TeleportAbsolute
from turtlesim.srv import SetPen

def get_my_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def set_random_color():
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return r, g, b

class NodoTortugaClean(Node):
    def __init__(self):
        super().__init__('d_nodo_tortuga_clean')
        self.is_pen_down = True
        self.client_set_pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.declare_parameter('speed', 2.0)
        self.speed = self.get_parameter('speed').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info("\n'WASD' para mover."
                               "\n'R' para resetear la posición."
                               "\n'C' para borrar trazos."
                               "\n'Q' para salir.")

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
                angular_speed = 0.0  # Detener giro

            if key == 'r': # Reset
                self.reset_turtle()
                continue

            if key == 'c': # Clear trazos
                self.clear_traces()
                continue

            if key == 'q': # Salir
                break

            if key == ' ':
                self.is_pen_down = not self.is_pen_down
                r, g, b = set_random_color()
                set_pen_req = SetPen.Request()
                set_pen_req.r = r
                set_pen_req.g = g
                set_pen_req.b = b
                set_pen_req.width = 3
                set_pen_req.off = 0 if self.is_pen_down else 1  # 0 para bajar, 1 para levantar
                self.client_set_pen.call_async(set_pen_req)  # Actualiza el lápiz

            twist.linear.x = linear_speed
            twist.angular.z = angular_speed

            self.publisher_.publish(twist)

    def reset_turtle(self):
        # Clientes para los servicios necesarios
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        # Esperar a que los servicios estén disponibles
        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /turtle1/teleport_absolute...')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando al servicio /turtle1/set_pen...')

        set_pen_req = SetPen.Request()
        set_pen_req.width = 3  # Ancho del lápiz
        set_pen_req.off = 1  # Levantar el lápiz
        set_pen_client.call_async(set_pen_req)

        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5

        teleport_client.call_async(req)
        self.get_logger().info('Posición reiniciada.')

        set_pen_req.off = 0 # Bajar el lápiz
        set_pen_client.call_async(set_pen_req)

    def clear_traces(self):
        # Cliente para el servicio de limpieza
        client_clear = self.create_client(Empty, '/clear')
        while not client_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio /clear...')

        req = Empty.Request()
        client_clear.call_async(req)
        self.get_logger().info('Trazos borrados.')

def main(args=None):
    rclpy.init(args=args)
    node = NodoTortugaClean()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
