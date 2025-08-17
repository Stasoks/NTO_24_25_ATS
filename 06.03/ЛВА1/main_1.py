from mcx import MCX
import cv2
import numpy as np
import time
import sys
import asyncio
import os

def readWarning(robot):
    if robot.getManipulatorWarning() == 0:
        return True
    else:
        return False
    
def getData(robot):
    if not robot.getManipulatorMotor() == []:
        print(f"\nMotor: {robot.getManipulatorMotor()}")
        print(f"Temperature: {robot.getManipulatorTemperature()}")
        print(f"Load: {robot.getManipulatorLoad()}\n")
        return True
    else:
        print("Restart Server.py")
        return False

async def motion_programm(robot, step, sample_number):
    samples = {
        0: [450, 400, 280], 1: [450, -400, 280], 2: [550, 460, 280],
        3: [550, 360, 280], 4: [550, -300, 280], 5: [550, -400, 280],
        6: [650, 460, 280], 7: [650, 360, 280], 8: [650, 260, 280],
        9: [650, -200, -280], 10: [650, -300, -280], 11: [650, -400]
    }

    start_point = [489, -135, 350]
    camera_point = [670, -511, 311.5]

    sample_coords = samples.get(sample_number)
    
    match(step):
        case(0): 
            # Стартовая точка
            robot.move("Robot3_1", start_point[0], start_point[1], start_point[2], 0, 0)
            await asyncio.sleep(3)  # Задержка 5 секунд
            return True

        case(1):
            # Перемещение к точке пробы (над пробой)
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 250, 0, 0)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(2):
            # Опускание к пробе
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 95, 0, 0)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True
        
        case(3):
            # Захват пробы
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 95, 0, 1)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(4):
            # Поднятие пробы
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 250, 0, 1)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(5):
            # Перемещение к камере
            robot.move("Robot3_1", camera_point[0], camera_point[1], camera_point[2], 0, 1)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(6):
            print("Проба доставлена к камере. Начинаем запись с камеры.")
            
            # Создаем папку для фотографий, если она не существует
            if not os.path.exists("photos"):
                os.makedirs("photos")

            # Получение изображения с камеры
            image_byte = robot.getCamera1Image()
            if image_byte:
                image_np = np.frombuffer(image_byte, np.uint8)
                image_np = cv2.rotate(image_np, cv2.ROTATE_180)
                image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
                
                # Сохранение фотографии
                filename = f"photos/sample_{sample_number}_camera_capture.jpg"
                cv2.imwrite(filename, image_np)
                print(f"Фотография сохранена: {filename}")
                
                # Отображение изображения
                cv2.imshow("Camera Feed", image_np)
                cv2.waitKey(1) 
                await asyncio.sleep(5)  # Задержка для отображения
                cv2.destroyAllWindows()
            await asyncio.sleep(7)
            return True

        case(7):
            # Возвращение к точке пробы (над пробой)
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 250, 0, 1)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(8):
            # Опускание пробы
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 95, 0, 1)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(9):
            # Отпускание пробы
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 95, 0, 0)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(10):
            # Поднятие манипулятора 
            robot.move("Robot3_1", sample_coords[0], sample_coords[1], 250, 0, 0)
            await asyncio.sleep(2)  # Задержка 1 секунда
            return True

        case(11):
            # Возвращение в стартовую точку
            robot.move("Robot3_1", start_point[0], start_point[1], start_point[2], 0, 0)
            await asyncio.sleep(3)  # Задержка 6 секунд
            return True
        
        case(12):
            print("Программа завершена!")
            sys.exit(0)

async def main():
    print("start")
    robot = MCX() 
    await asyncio.sleep(1) 
    programm_start_bool = True
    current_count = 0
    start_count = robot.getManipulatorCount()
    step = 0  # Инициализация переменной step
    sample_number = 7  # Задаем номер пробы вручную

    while programm_start_bool:
        current_count = robot.getManipulatorCount()
        if robot.getManipulatorStatus() == 0:
            print(f"Step {step}")
            if await motion_programm(robot, step, sample_number):
                step += 1  # Переходим к следующему шагу только если текущий шаг завершен
            await asyncio.sleep(0.1)  # Небольшая задержка для снижения нагрузки на CPU

        if robot.getManipulatorStatus() == 1:
            print("Robot move")

        if readWarning(robot):
            print("No error")
            getData(robot)
        else:
            print(robot.getManipulatorWarningStr())
            print(f"In step {step}")
            exit()
                                
        await asyncio.sleep(0.01)  # Небольшая задержка для снижения нагрузки на CPU

if __name__ == "__main__":
    asyncio.run(main())  # Запуск асинхронной программы