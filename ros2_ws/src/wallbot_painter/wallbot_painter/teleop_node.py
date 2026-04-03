import sys
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
Interface de Contrôle du Wallbot Painter
-----------------------------------------
Déplacement:
        w
   a    s    d
        x

w/x : augmenter/diminuer la vitesse linéaire
a/d : augmenter/diminuer la vitesse angulaire
s : arrêt complet

Peinture:
   p : Activer/Désactiver la peinture (toggle)

Réglages:
   q/z : +/- vitesse linéaire max de 10%
   e/c : +/- vitesse angulaire max de 10%

CTRL-C pour quitter
"""

move_bindings = {
    'w': (1, 0),
    'a': (0, 1),
    'd': (0, -1),
    'x': (-1, 0),
}

speed_bindings = {
    'q': (1.1, 1.0),
    'z': (.9, 1.0),
    'e': (1.0, 1.1),
    'c': (1.0, .9),
}

def get_key(settings):
    if sys.platform == 'win32':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def save_settings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restore_settings(settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

class TeleopWallbot(Node):
    def __init__(self):
        super().__init__('teleop_wallbot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.paint_publisher_ = self.create_publisher(Bool, 'painting_active', 10)
        
        self.speed = 0.1  # m/s
        self.turn = 0.5   # rad/s
        self.painting = False
        
        self.get_logger().info("Teleop Wallbot initié.")

    def update_v(self, x, th):
        twist = Twist()
        twist.linear.x = x * self.speed
        twist.angular.z = th * self.turn
        self.publisher_.publish(twist)

    def toggle_paint(self):
        self.painting = not self.painting
        msg = Bool()
        msg.data = self.painting
        self.paint_publisher_.publish(msg)
        return self.painting

def main():
    settings = save_settings()
    rclpy.init()
    node = TeleopWallbot()

    x = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        print(f"Vitesse actuelle: speed {node.speed} | turn {node.turn} | Peinture: {'ON' if node.painting else 'OFF'}")
        
        while True:
            key = get_key(settings)
            
            if key in move_bindings.keys():
                x = move_bindings[key][0]
                th = move_bindings[key][1]
            elif key in speed_bindings.keys():
                node.speed *= speed_bindings[key][0]
                node.turn *= speed_bindings[key][1]
                print(f"Vitesse actuelle: speed {node.speed:.2f} | turn {node.turn:.2f}")
            elif key == 's':
                x = 0.0
                th = 0.0
            elif key == 'p':
                active = node.toggle_paint()
                print(f"Peinture: {'ON' if active else 'OFF'}")
            elif key == '\x03': # CTRL-C
                break
            else:
                # Si on lâche ou autre touche, on ne change pas x/th forcement ici
                # mais pour un teleop standard on peut rester sur la derniere commande
                pass
            
            node.update_v(x, th)

    except Exception as e:
        print(e)

    finally:
        # Stopper le robot avant de quitter
        node.update_v(0.0, 0.0)
        restore_settings(settings)
        rclpy.shutdown()

if __name__ == '__main__':
    main()
