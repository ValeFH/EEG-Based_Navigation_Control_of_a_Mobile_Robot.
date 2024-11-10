"""Librerias"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from ras_tutorial_interfaces.msg import Sphere
from geometry_msgs.msg import Twist
from time import sleep
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sensor_msgs.msg import LaserScan
import time
import threading
from collections import Counter

"""Funciones para selección de ruta"""

# Verifica que el nuevo indice sea diferente a todos los indices almacenados en la matriz.
def todos_diferentes_indices(indices, valor):
    # Almacena el numero de filas y columnas de la matriz.
    num_filas, num_columnas = indices.shape
    # Recorre todas las filas de la matriz.
    for fila in range(num_filas):
        # Recorre todas las columnas de la matriz.
        for columna in range(num_columnas):
            # Verifica si el indice actual coincide con el indice almacenado en dicha posicion.
            if indices[fila][columna] == valor:
                return False
    return True

# Selecciona las señales para cumplir la ruta deseada.
def salida_laberinto(x_test,y_test):
    # Selección de ruta deseada.
    # Ruta 1
    #ruta = [0,1,1,1,1,0,3,3,0,2,2,0,3,3,3,3,0,2,2,0,3,3,3,0,2,2,0,3,0]
    # Ruta 2
    ruta = [0,2,2,0,3,3,0,1,1,0,3,3,3,3,3,0,2,2,0,3,3,3,0,1,1,0,3,3]

    # Contar cuentas veces se repite cada número en la lista
    conteo = Counter(ruta)
    # Encontrar el número que mas se repite.
    numero_mas_comun, max_repeticiones = conteo.most_common(1)[0]

    #Inicialización de variables.
    x_camino = [] # Almacena las señales necesarias para cumplir la ruta deseada
    y_camino = []  # Almacena los indicadores para cumplir la ruta deseada
    eventos = 4 # Numero de eventos posibles.
    indices = [[1000 for _ in range(eventos)] for _ in range(max_repeticiones)] # Crear la matriz con el valor 1000 en cada posición
    filas  =  [0,0,0,0] # Almacena el indice de la señal guardada.
    
    # Convierte las matrices en arreglos numpy
    indices = np.array(indices)
    filas = np.array(filas)

    # Recorre todos los indicadores de la ruta.
    for i in range(len(ruta)):
        # Recorre la matriz con los indicadores de la señal de entrada.
        for j in range(len(y_test)):
            # Verifica si el indicador de la matriz de entrada coincide con el indicador actual de la ruta.
            if y_test[j] == ruta[i]:
                # Verifica que esa señal no se usara anteriormente.
                if (todos_diferentes_indices(indices,j)):
                    # Almacena el indicador y la señal seleccionada.
                    x_camino.append(x_test[j])
                    y_camino.append(y_test[j])
                    # Verifica el indicador a que accion corresponde, lo almacen en la columna correspondiente
                    # e incrementa dicha fila
                    if(y_test[j]==0):
                        indices[filas[0]][0] = j
                        filas[0] = filas[0] + 1
                    elif(y_test[j]==1):
                        indices[filas[1]][1] = j
                        filas[1] = filas[1] + 1
                    elif(y_test[j]==2):
                        indices[filas[2]][2] = j
                        filas[2] = filas[2] + 1
                    else:
                        indices[filas[3]][3] = j
                        filas[3] = filas[3] + 1
                    break

    # Convierte las matrices en arreglos numpy                 
    x_camino = np.array(x_camino)
    y_camino = np.array(y_camino)
    
    return x_camino, y_camino 

"""Clase del nodo"""

# Permite segun las señales EEG procesadas, realizar la prediccion de la accion que debe realizar
# el robot con el modelo de inteligencia artificial y enviar los mensajes necesarios para poder 
# llevar a cabo dicho movimiento. Ademas se incluye la parada de emergencia para evitar choques
class RobotEEG(Node):
    # Constructor de la clase, inicialización de parametros, publicadores, subscriptores y temporizadores. 
    def __init__(self, x_camino, y_camino, modelo_IA):

        # Inicialización del nodo con el nombre 'robot_EEG'
        super().__init__('robot_EEG')
        
        # Publicador y subscriptor
        self.publicador_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10) # Comandos de velocidad angular y lineal
        self.subscriptor_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10) # Valores Lidar

        # Atributos  
        self.mensaje_cmd_vel = Twist() # Mensaje topico /cmd_vel.
        self.safe_to_move = True # Indicador parada de emergencia.
        self.vel_lineal = 1.1 # Velocidad del robot para avanzar.
        self.x_camino = x_camino # Señales para completar la ruta. 
        self.y_camino = y_camino # Indicadores de la ruta.
        self.modelo_IA = modelo_IA # Modelo de inteligencia artificial.
        self.i = 0 # Indicador actual de la ruta definida.
        self.y_estimado = 3 # Salida estimada por el modelo de IA.
        self.move_start_time = None  # Tiempo de inicio del movimiento
        self.direccion = {0: "reposo", 1: "giro izquierda", 2: "giro derecha", 3: "avanzar"} # Diccionario de movimientos

        # Configuracion timer o funciones recurrentes.
        self.timer = self.create_timer(0.01, self.timer_callback_method)

    # Método callback que se llama a intervalos regulares para controlar los movimientos del robot.
    def timer_callback_method(self):
        # Verifica si hay movimiento en progreso.
        if self.move_start_time is None:  
            # Registrar el tiempo en el que inicio el movimiento
            self.move_start_time = time.time()               

            # Extrae la señal de entrada del indicador de la ruta actual.  
            fila = self.x_camino[self.i]

            # Realiza la prediccion con el modelo de inteligencia artificial. 
            self.y_estimado = self.modelo_IA.predict(fila.reshape(1, -1))

            # Identifica si la salida estimada corresponde a la salida esperada e imprime el movimiento.
            if self.y_estimado == self.y_camino[self.i]:
                self.get_logger().info(f'Identificado, accion: {self.direccion[self.y_estimado[0]]}, Predicción: {self.y_estimado[0]}, Valor real: {self.y_camino[self.i]}')
            else:
                self.get_logger().info(f'No identificado, accion: {self.direccion[self.y_estimado[0]]}, Predicción: {self.y_estimado[0]}, Valor real: {self.y_camino[self.i]}')

            # Asignar velocidades según el valor de y_estimado
            if self.y_estimado == 0:
                # Reposo - Quieto
                self.mensaje_cmd_vel.linear.x = 0.0
                self.mensaje_cmd_vel.angular.z = 0.0
            elif self.y_estimado == 1:
                # Puño izquierdo - izquierda
                self.mensaje_cmd_vel.linear.x = 0.0
                self.mensaje_cmd_vel.angular.z = 0.3925
            elif self.y_estimado == 2:
                # Puño derecho - derecha
                self.mensaje_cmd_vel.linear.x = 0.0
                self.mensaje_cmd_vel.angular.z = -0.3925
            else:
                # Ambos puños - avanzar
                self.mensaje_cmd_vel.linear.x = self.vel_lineal
                self.mensaje_cmd_vel.angular.z = 0.0

            # Publicar el mensaje
            self.publicador_cmd_vel.publish(self.mensaje_cmd_vel)

        else:
            # Si han pasado 2 segundos desde el inicio del movimiento, detener el robot.
            if time.time() - self.move_start_time >= 2:
                self.mensaje_cmd_vel.linear.x = 0.0
                self.mensaje_cmd_vel.angular.z = 0.0
                # Publicar el mensaje
                self.publicador_cmd_vel.publish(self.mensaje_cmd_vel)
                # Reinicio para el siguiente movimiento.
                self.move_start_time = None 

                # Actualizar el índice para la siguiente iteración (siguiente indicador de la ruta)
                if self.i < (len(self.y_camino) - 1):
                    self.i += 1
                else:
                    self.i = 0
                    self.get_logger().info('Fin de la ruta')  

    # Método callback que se llama cuando se recibe un mensaje en el suscriptor
    # Verificar si hay algún obstáculo cercano para activar la parada de emergencia.
    def lidar_callback(self, msg: LaserScan):
        # Almacena el valor del sensor en el angulo correspondiente al frente del robot.
        distancia_actual = msg.ranges[1]
        # Umbral para decision de frenado.
        umbral = 1.2
        # Verifica si la distancia actual al objeto es mayor o igual al umbral.
        self.safe_to_move = distancia_actual >= umbral
        # Si la distancia actual es menor al umbral y se encuentra avanzando, 
        # activar parada de emergencia.
        if (not self.safe_to_move) and (self.y_estimado == 3) :
            self.mensaje_cmd_vel.linear.x = 0.0
            self.mensaje_cmd_vel.angular.z = 0.0
            # Publicar el mensaje
            self.publicador_cmd_vel.publish(self.mensaje_cmd_vel)
            # Enviar mensaje del estado del nodo.
            self.get_logger().info('Obstáculo detectado. Parada de emergencia activada.')        

def main(args=None):
    # Inicializar
    rclpy.init(args=args)

    # Cargar modelos y datos
    # Base externa
    """
    modelo_IA = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_externa/IA_model.pkl')
    scaler = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_externa/scaler.pkl')
    x_test = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_externa/X_test.pkl')
    y_test = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_externa/y_test.pkl')
    """
    # Base propia
    modelo_IA = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_propia/IA_model.pkl')
    scaler = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_propia/scaler.pkl')
    x_test = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_propia/X_test.pkl')
    y_test = joblib.load('/home/majocabra/Desktop/Tesis_2/robot_EEG_ws/src/ras_topics/ras_topics/modelo_base_propia/y_test.pkl')
    
    # Seleccion de ruta.
    x_camino, y_camino = salida_laberinto(x_test,y_test)

    # Crear instancia del nodo.
    robot_EEG = RobotEEG(x_camino, y_camino,modelo_IA)

    # Ejecutar el nodo en un bucle.
    rclpy.spin(robot_EEG)
    
    # Al finalizar destruir el nodo y cerrar
    robot_EEG.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()