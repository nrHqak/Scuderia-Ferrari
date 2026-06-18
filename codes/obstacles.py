#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor
from pybricks.parameters import Port, Color, Button
from pybricks.iodevices import I2CDevice
from pybricks.tools import wait, StopWatch
import math

# --- ИНИЦИАЛИЗАЦИЯ ---
ev3 = EV3Brick()
ev3.light.on(Color.ORANGE)

chap = UltrasonicSensor(Port.S1)    # Левый УЗ
rast = UltrasonicSensor(Port.S2)    # Правый УЗ
color_sensor = ColorSensor(Port.S3) # Датчик цвета в пол

motor_b = Motor(Port.A) # Ходовой мотор
motor_a = Motor(Port.B) # Рулевой мотор (Аккерман)

try:
    pixy = I2CDevice(Port.S4, 0x54)
    print("Pixy2.1: OK")
except:
    print("Pixy2.1: ERROR!")

motor_a.reset_angle(0)
motor_b.reset_angle(0)

print("Ждем старт...")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

ev3.light.on(Color.GREEN)
ev3.speaker.beep(frequency=400, duration=200)
wait(200)
print("СТАРТ")

# --- НАСТРОЙКИ ---
speed = 800 # Скорость

abi = [Color.BLUE, Color.BLACK]
narengi = [Color.ORANGE, Color.RED]

direction = None
line_counter = 0

# Фильтры УЗ истории
left_history = [30.0, 30.0, 30.0]
right_history = [30.0, 30.0, 30.0]
filtered_target = 0.0

blue_count = 0
orange_count = 0

# --- СЛУЖЕБНЫЕ ФУНКЦИИ ---
def clamp(value, mn, mx):
    if value > mx: return mx
    if value < mn: return mn
    return value

def median3(arr):
    s = sorted(arr)
    return s[1]

def get_left():
    global left_history
    d = chap.distance() / 10.0
    if 5.0 < d < 150.0:
        left_history.append(d)
        left_history.pop(0)
    return median3(left_history)

def get_right():
    global right_history
    d = rast.distance() / 10.0
    if 5.0 < d < 150.0:
        right_history.append(d)
        right_history.pop(0)
    return median3(right_history)

def amotor(target_angle, max_speed=800):
    clamped_angle = clamp(target_angle, -35, 35) # Максимальный выворот ограничен 35 градусами
    motor_a.run_target(max_speed, clamped_angle, wait=False)

def smooth(value):
    global filtered_target
    filtered_target = filtered_target * 0.7 + value * 0.3
    return filtered_target

def parse_pixy_blocks():
    request = bytes([174, 193, 32, 2, 3, 5])
    try:
        pixy.write(0, request)
        reply = pixy.read(0, 20)
        if reply[0] == 175 and reply[1] == 193:
            sig = reply[7] << 8 | reply[6]
            x_val = reply[9] << 8 | reply[8]
            y_val = reply[11] << 8 | reply[10]
            width = reply[12]
            if width > 10 and sig <= 7:
                return sig, x_val, y_val
    except:
        pass
    return 0, 157, 0

# ==================================================
# ЭТАП 1: ПОИСК ПЕРВОЙ ЛИНИИ (ОПРЕДЕЛЕНИЕ НАПРАВЛЕНИЯ)
# ==================================================
print("Поиск первой линии...")

while direction is None:
    motor_b.run(speed)
    r = get_right()
    c = get_left()

    fr = (-2 * math.sqrt(11 * r)) + 100
    fc = (-2 * math.sqrt(11 * c)) + 100
    target = (fc * 1.3) - (fr * 1.3)
    
    amotor(smooth(target))

    color = color_sensor.color()
    if color in abi: blue_count += 1
    else: blue_count = 0

    if color in narengi: orange_count += 1
    else: orange_count = 0

    if blue_count >= 3:
        direction = "LEFT"
        line_counter = 1
        motor_b.reset_angle(0)
        print("Направление LEFT")
        break

    if orange_count >= 3:
        direction = "RIGHT"
        line_counter = 1
        motor_b.reset_angle(0)
        print("Направление RIGHT")
        break

    wait(5)

# ==================================================
# ЭТАП 2: ОСНОВНОЙ ЗАЕЗД (С ПРЕПЯТСТВИЯМИ)
# ==================================================
print("Основной цикл")

while True:
    motor_b.run(speed)

    # 1. СЧЕТЧИК ЛИНИЙ
    color = color_sensor.color()
    if color in abi: blue_count += 1
    else: blue_count = 0

    if color in narengi: orange_count += 1
    else: orange_count = 0

    line_detected = (blue_count >= 3 or orange_count >= 3)

    if motor_b.angle() > 1000 and line_detected:
        line_counter += 1
        motor_b.reset_angle(0)
        print("Линия:", line_counter)
        blue_count = 0
        orange_count = 0

    # Если нужно проехать ровно 1 круг (в зависимости от разметки, обычно это 4, 8 или 12 линий)
    # Поменяй число 12 на то, которое у тебя означает финиш.
    if line_counter == 12: 
        break

    # 2. ОПРОС КАМЕРЫ PIXY
    sig, x_val, y_val = parse_pixy_blocks()
    if y_val < 35: 
        sig = 0 # Игнорируем столбы, которые слишком далеко (на горизонте)

    # 3. ЛОГИКА РУЛЕНИЯ (УЛЬТРАЗВУК + КАМЕРА)
    if sig == 1:
        # ВИДИМ КРАСНЫЙ СТОЛБ: Объезжаем справа (столб должен быть слева в кадре)
        error = x_val - 45 # Идеальный X для красного = 45
        target = error * 1.5 # Жесткий коэффициент для максимального выворота
        amotor(smooth(target))

    elif sig == 2:
        # ВИДИМ ЗЕЛЕНЫЙ СТОЛБ: Объезжаем слева (столб должен быть справа в кадре)
        error = x_val - 270 # Идеальный X для зеленого = 270
        target = error * 1.5 
        amotor(smooth(target))

    else:
        # СТОЛБОВ НЕТ: Едем по стенам (Это предотвратит инверсию на поворотах и вернет руль в ноль!)
        if direction == "LEFT":
            distance = get_left()
            target = (distance - 28) * -2
            amotor(smooth(target))
        elif direction == "RIGHT":
            distance = get_right()
            target = (distance - 28) * 2
            amotor(smooth(target))

    wait(5)

# ==================================================
# ЭТАП 3: ДОЕЗД ДО ФИНИША И ОСТАНОВКА
# ==================================================
print("Финальный участок")
timer = StopWatch()
timer.reset()

while timer.time() < 600:
    motor_b.run(speed)
    if direction == "LEFT":
        distance = get_left()
        target = (distance - 28) * -2
        amotor(smooth(target))
    else:
        distance = get_right()
        target = (distance - 28) * 2
        amotor(smooth(target))
    wait(5)

# ФИНИШ
motor_b.stop()
motor_a.run_target(600, 0) # Ровняем колеса
ev3.light.on(Color.RED)
print("Заезд завершен")