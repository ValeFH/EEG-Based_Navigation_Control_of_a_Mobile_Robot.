import rclpy
from std_msgs.msg import Float64
from ras_tutorial_interfaces.msg import Sphere
from geometry_msgs.msg import Twist

# 2. Crear la funcion que permita leer el mensaje recibido
def recibir_mensaje(msg):
    if(msg.data == 1):
        print('Identificado')
    else:
        print('No identificado')

def recibir_cmd_vel(msg:Twist):
    print(f'Cmd_vel-> x: {msg.linear.x}, angular z: {msg.angular.z}')    


def main():
    # Inicializar
    rclpy.init()
    # Crear el nodo
    node = rclpy.create_node('subscriptor')
    # Subscriptor
    # 1. Crear el entorno del subscriptor
    subscriptor = node.create_subscription(Float64, 'mensajes', recibir_mensaje, 10)
    subscriptor3 = node.create_subscription(Twist, '/cmd_vel', recibir_cmd_vel, 10)
    # 3. No destruir el programa
    rclpy.spin(node)
    # Al finalizar
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()