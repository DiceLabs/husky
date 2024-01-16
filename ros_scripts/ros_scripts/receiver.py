import socket

def receive_file(filename, host, port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((host, port))
        s.listen()
        print(f"Listening for incoming file at {host}:{port}...")

        conn, addr = s.accept()
        print(f"Connected by: {addr}")

        with open(filename, 'wb') as file:
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                file.write(data)

    print(f"File received and saved as '{filename}'.")

if __name__ == "__main__":
    file_to_save = "/home/philip/environment"
    host = "146.244.98.21"   # Change this to the receiver's IP address or leave it as "0.0.0.0" to listen on all available interfaces
    port = 12345       # Change this to the same port used in the sender script
    receive_file(file_to_save, host, port)
