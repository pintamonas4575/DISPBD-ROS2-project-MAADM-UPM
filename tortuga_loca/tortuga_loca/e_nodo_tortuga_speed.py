import rclpy
import sys, termios, tty
import threading
import random
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen

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

class NodoTortugaSpeed(Node):
    def __init__(self):
        super().__init__('e_nodo_tortuga_speed')
        self.is_pen_down = True
        self.client_set_pen = self.create_client(SetPen, '/turtle1/set_pen')

        self.declare_parameter('speed', 2.0)
        self.speed = self.get_parameter('speed').value
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.get_logger().info("\n'WASD' para mover."
                               "\n'R' para resetear la posición."
                               "\n'C' para borrar trazos."
                               "\n'Espacio' para pintar/despintar."
                               "\n'Q' para salir."
                               "\n--Cualquier otra tecla para parar la tortuga--")

        self.add_on_set_parameters_callback(self.parameter_callback)

        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.stop_node = False

        # Hilo para capturar el teclado
        self.keyboard_thread = threading.Thread(target=self.capture_keys, daemon=True)
        self.keyboard_thread.start()

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'speed':
                self.speed = param.value  # Actualizar velocidad
                self.get_logger().info(f"Velocidad actualizada desde la terminal: {self.speed}")
        return SetParametersResult(successful=True)

    def capture_keys(self):
        # Hilo para capturar las teclas del usuario
        while rclpy.ok() and not self.stop_node:
            key = get_my_key()

            if key.lower() == 'w':
                self.linear_speed = self.speed
            elif key.lower() == 's':
                self.linear_speed = -self.speed
            elif key.lower() == 'a':
                self.angular_speed = self.speed
            elif key.lower() == 'd':
                self.angular_speed = -self.speed
            elif key.lower() == ' ':
                self.toggle_pen()
            elif key.lower() == 'r':
                self.reset_turtle()
            elif key.lower() == 'c':
                self.clear_traces()
            elif key.lower() == 'q':
                self.stop_node = True
                break
            else: # Cualquier otra tecla, detener tortuga
                self.linear_speed = 0.0
                self.angular_speed = 0.0

    def toggle_pen(self):
        self.is_pen_down = not self.is_pen_down
        r, g, b = set_random_color()
        set_pen_req = SetPen.Request()
        set_pen_req.r = r
        set_pen_req.g = g
        set_pen_req.b = b
        set_pen_req.width = 3
        set_pen_req.off = 0 if self.is_pen_down else 1
        self.client_set_pen.call_async(set_pen_req)

    def run(self):
        # Ciclo principal del nodo
        twist = Twist()

        while rclpy.ok() and not self.stop_node:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        rclpy.shutdown()

    def reset_turtle(self):
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

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

        set_pen_req.off = 0  # Bajar el lápiz
        set_pen_client.call_async(set_pen_req)

    def clear_traces(self):
        client_clear = self.create_client(Empty, '/clear')
        while not client_clear.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando por el servicio /clear...')

        req = Empty.Request()
        client_clear.call_async(req)
        self.get_logger().info('Trazos borrados.')

def main(args=None):
    rclpy.init(args=args)
    node = NodoTortugaSpeed()
    node.run()
    node.destroy_node()

if __name__ == '__main__':
    main()
