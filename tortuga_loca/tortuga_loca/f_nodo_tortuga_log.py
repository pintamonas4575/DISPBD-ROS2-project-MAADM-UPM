import rclpy
import sys, termios, tty
import threading
import random
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute, SetPen

def get_my_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def set_random_color() -> tuple[int, int, int]:
    r = random.randint(0, 255)
    g = random.randint(0, 255)
    b = random.randint(0, 255)
    return r, g, b

class NodoTortugaSpeed(Node):
    def __init__(self):
        super().__init__('f_nodo_tortuga_log')
        self.is_pen_down = True
        self.client_set_pen = self.create_client(SetPen, '/turtle1/set_pen')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Parámetro de velocidad
        self.declare_parameter('speed', 2.0)
        self.speed = self.get_parameter('speed').value

        # Parámetro de nivel de logging
        self.declare_parameter('log_level', 'INFO')
        self.log_level = self.get_parameter('log_level').value
        self.set_logger_level(self.log_level)

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

    def parameter_callback(self, params) -> SetParametersResult:
        for param in params:
            if param.name == 'speed':
                self.speed = param.value
                self.dinamic_log(self.log_level,f"Velocidad actualizada desde la terminal: {self.speed}")
            if param.name == 'log_level':
                self.set_logger_level(param.value)
                self.log_level = param.value
                self.dinamic_log(self.log_level,f"Log level actualizado desde la terminal: {self.log_level}")
        return SetParametersResult(successful=True)

    def set_logger_level(self, log_level: str) -> None:
        log_levels = {
            'DEBUG': LoggingSeverity.DEBUG,
            'INFO': LoggingSeverity.INFO,
            'WARN': LoggingSeverity.WARN,
            'ERROR': LoggingSeverity.ERROR,
            'FATAL': LoggingSeverity.FATAL,
        }

        level = log_levels.get(log_level.upper(), LoggingSeverity.INFO)
        self.get_logger().set_level(level)

    def dinamic_log(self, level: str, message: str) -> None:
        if level == 'DEBUG':
            self.get_logger().debug(message)
        elif level == 'INFO':
            self.get_logger().info(message)
        elif level == 'WARN':
            self.get_logger().warn(message)
        elif level == 'ERROR':
            self.get_logger().error(message)
        elif level == 'FATAL':
            self.get_logger().fatal(message)
        else:
            self.get_logger().info(message)  # Por defecto

    def capture_keys(self) -> None:
        # Hilo para capturar las teclas
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

    def run(self) -> None:
        # Ciclo principal del nodo
        twist = Twist()

        while rclpy.ok() and not self.stop_node:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed

            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        rclpy.shutdown()

    def toggle_pen(self) -> None:
        self.is_pen_down = not self.is_pen_down
        r, g, b = set_random_color()
        set_pen_req = SetPen.Request()
        set_pen_req.r = r
        set_pen_req.g = g
        set_pen_req.b = b
        set_pen_req.width = 3
        set_pen_req.off = 0 if self.is_pen_down else 1
        self.client_set_pen.call_async(set_pen_req)

    def reset_turtle(self) -> None:
        teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        while not teleport_client.wait_for_service(timeout_sec=1.0):
            self.dinamic_log(self.log_level,'Esperando al servicio /turtle1/teleport_absolute...')
        while not set_pen_client.wait_for_service(timeout_sec=1.0):
            self.dinamic_log(self.log_level,'Esperando al servicio /turtle1/set_pen...')

        set_pen_req = SetPen.Request()
        set_pen_req.width = 3  # Ancho del lápiz
        set_pen_req.off = 1  # Levantar el lápiz
        set_pen_client.call_async(set_pen_req)

        req = TeleportAbsolute.Request()
        req.x = 5.5
        req.y = 5.5
        teleport_client.call_async(req)
        self.dinamic_log(self.log_level, 'Posición reiniciada.')

        set_pen_req.off = 0  # Bajar el lápiz
        set_pen_client.call_async(set_pen_req)

        twist = Twist()
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher_.publish(twist) # Publico para parar la tortuga

    def clear_traces(self) -> None:
        client_clear = self.create_client(Empty, '/clear')
        while not client_clear.wait_for_service(timeout_sec=1.0):
            self.dinamic_log(self.log_level,'Esperando por el servicio /clear...')

        req = Empty.Request()
        client_clear.call_async(req)
        self.dinamic_log(self.log_level,'Trazos borrados.')

def main(args=None):
    rclpy.init(args=args)
    node = NodoTortugaSpeed()
    node.run()
    node.destroy_node()

if __name__ == '__main__':
    main()
