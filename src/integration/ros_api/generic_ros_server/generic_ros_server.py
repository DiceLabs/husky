import socket
from server_log import Logging
from defaults import Defaults
import pickle

FREQUENCY = 20
PERIOD = 1/FREQUENCY

def loop(sock, callback):
    conn, addr = sock.accept()
    with conn:
        Logging.log_connection_message(addr)
        data = conn.recv(Defaults.BUFFER_SIZE)
        if data:
            Logging.log_data_rcv_message(data)
            request = pickle.loads(data)
            response = callback(request)
            conn.sendall(pickle.dumps(response))

"""
    Generic Server API to Spawn ROS Server Process
"""
def start_ros_server(name=Defaults.DEFAULT_NAME, 
                host=Defaults.LOCALHOST, port=Defaults.PORT, 
                callback=Defaults.default_callback):
    import rospy
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind((host, port))
        sock.listen()
        Logging.log_server_active_message(name, host, port)
        loop(sock, callback)
        rospy.Timer(rospy.Duration(PERIOD), lambda event: loop(sock, callback))
        rospy.spin()

if __name__ == "__main__":
    start_ros_server()