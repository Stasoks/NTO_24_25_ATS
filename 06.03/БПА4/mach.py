import json, socket, time

ip_1 = 'localhost'
ip_2 = 'localhost'
data = []
with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((ip_1, 9092))
    while True:
        data = sock.recv(1024)
        if data:
            data = data   
            print(json.loads(data))
            time.sleep(0.01)
            break

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.connect((ip_2, 9093))
    sock.sendall(data)