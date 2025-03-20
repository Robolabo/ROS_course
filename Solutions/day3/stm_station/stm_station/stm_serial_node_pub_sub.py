# 1. Import necessary libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

# 2. Define the SerialPubSubNode class
class SerialPubSubNode(Node):
    def __init__(self):
        super().__init__('stm_serial_node')

        # 3. Configuración del puerto serial (actualiza el puerto correcto)
        self.port = '/dev/ttyACM0'  # Ejemplo: '/dev/ttyUSB0' en Linux
        self.baudrate = 115200
        self.loop_frequency = 1000

        # 4. Inicializar puerto serial
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=0.01)
        self.get_logger().info(f'Serial port {self.port} @ {self.baudrate} baud')

        # 5. Crear publisher para datos del STM
        self.stm_state_topic = 'stm_state'
        self.stm_state_publisher = self.create_publisher(
            Float32MultiArray, 
            self.stm_state_topic, 
            10
        )

        # 6. Timer para lectura de datos
        self.timer = self.create_timer(1/self.loop_frequency, self.timer_read_pub_callback)

        # 7. Crear subscriber para comandos de control
        self.stm_control_topic = 'stm_control'
        self.stm_control_subscriber = self.create_subscription(
            Float32MultiArray,
            self.stm_control_topic,
            self.stm_control_callback,
            10
        )

    def timer_read_pub_callback(self):
        if self.serial_port.in_waiting > 0:
            try:
                # Leer y decodificar datos
                serial_data_raw = self.serial_port.readline()
                serial_data_decoded = serial_data_raw.decode('utf-8').strip()
                
                # Validar formato
                if not serial_data_decoded.startswith('{') or not serial_data_decoded.endswith('}'):
                    self.get_logger().warn('Formato incorrecto: faltan llaves')
                    return
                
                # Procesar datos
                serial_data = serial_data_decoded[1:-1]
                float_values = serial_data.split(',')
                
                if len(float_values) != 8:
                    self.get_logger().warn(f'Datos incompletos: {len(float_values)}/8 valores')
                    return
                
                # Crear y publicar mensaje
                msg = Float32MultiArray()
                msg.data = [float(val.strip()) for val in float_values]
                self.stm_state_publisher.publish(msg)
                
                self.get_logger().info(f'Publicado: {msg.data}')

            except Exception as e:
                self.get_logger().error(f'Error: {str(e)}')

    def stm_control_callback(self, msg):
        try:
            # Validar formato del mensaje (4 valores esperados)
            if len(msg.data) != 4:
                self.get_logger().warn('Comando inválido: se requieren 4 valores')
                return
                
            # Formatear comando: {control_type,goal,Kp,PWM}
            control_data = f"{{{msg.data[0]},{msg.data[1]},{msg.data[2]},{msg.data[3]}}}\n"
            
            # Enviar por serial
            self.serial_port.write(control_data.encode('utf-8'))
            self.get_logger().info(f'Comando enviado: {control_data.strip()}')

        except Exception as e:
            self.get_logger().error(f'Error en control: {str(e)}')

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialPubSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
