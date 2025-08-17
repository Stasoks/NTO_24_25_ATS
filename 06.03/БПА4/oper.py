import json, socket, time, cv2


with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(('', 9093))
    sock.listen(1)
    conn, addr = sock.accept()
    print('connected:', addr)
    while True:
        data = conn.recv(1024)
        if data:     
            data = json.loads(data)
            print(data)
            break



image = cv2.imread('/home/user/nto_solution/task_4/map.png')
print(image)
font = cv2.FONT_HERSHEY_SIMPLEX
for point in data['points']:
    image = cv2.rectangle(image, (point['tl'][0], point['tl'][1]), (point['br'][0], point['br'][1]), color=(0,0,255), thickness=2)
    image = cv2.putText(image, 'number: ' + str(point['number']) + " type: " + str(point['type']), (point['tl'][0]-100, point['tl'][1]-15), font, 
                    0.5, (255, 0, 0), 2, cv2.LINE_AA)

cv2.imshow('a', image)
cv2.waitKey(0)

