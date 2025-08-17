import json, socket, time


data = {'points':
    [{'number': 0,
    'type':0,
    'tl': (1150, 178),
    'br': (1185, 237)},
    {'number': 1,
    'type':2,
    'tl': (785, 533),
    'br': (817, 590)}
    ],
    }

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(('', 9092))
    sock.listen(1)
    conn, addr = sock.accept()
    print('connected:', addr)
    data = json.dumps(data).encode('utf-8')
    conn.sendall(data)