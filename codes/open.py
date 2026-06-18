#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick

from pybricks.ev3devices import Motor, UltrasonicSensor, ColorSensor

from pybricks.parameters import Port, Color, Button

from pybricks.tools import wait, StopWatch

import math



# --- ИНИЦИАЛИЗАЦИЯ ---

ev3 = EV3Brick()

ev3.light.on(Color.ORANGE)



chap = UltrasonicSensor(Port.S1) # Левый УЗ

rast = UltrasonicSensor(Port.S2) # Правый УЗ

color_sensor = ColorSensor(Port.S3) # Датчик цвета в пол



motor_b = Motor(Port.A) # Ходовой мотор (Дифференциал)

motor_a = Motor(Port.B) # Рулевой мотор (Аккерман)



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

speed = 800 # Базовая скорость в градусах/сек для Pybricks (~80% от макс)



abi = [Color.BLUE, Color.BLACK]

narengi = [Color.ORANGE, Color.RED]



direction = None

line_counter = 0



# Фильтры УЗ истории (в см)

left_history = [30.0, 30.0, 30.0]

right_history = [30.0, 30.0, 30.0]



# Сглаживание руления

filtered_target = 0.0



# Подтверждение цвета

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

# Переводим мм из Pybricks в см

d = chap.distance() / 10.0


if 5.0 < d < 150.0:

left_history.append(d)

left_history.pop(0)


return median3(left_history)



def get_right():

global right_history

# Переводим мм из Pybricks в см

d = rast.distance() / 10.0


if 5.0 < d < 150.0:

right_history.append(d)

right_history.pop(0)


return median3(right_history)



def amotor(target_angle, max_speed=700):

"""

В Pybricks run_target с флагом wait=False плавно удерживает

заданный угол, не блокируя выполнение основного цикла.

"""

clamped_angle = clamp(target_angle, -35, 35)

motor_a.run_target(max_speed, clamped_angle, wait=False)



def smooth(value):

global filtered_target

filtered_target = filtered_target * 0.7 + value * 0.3

return filtered_target



# ==================================================

# ЭТАП 1: ПОИСК ПЕРВОЙ ЛИНИИ

# ==================================================

print("Поиск первой линии...")



while direction is None:

motor_b.run(speed)



r = get_right()

c = get_left()



fr = (-2 * math.sqrt(11 * r)) + 100

fc = (-2 * math.sqrt(11 * c)) + 100



target = (fc * 1.3) - (fr * 1.3)

target = clamp(target, -28, 28)



# Применяем твое сглаживание фильтра низких частот

amotor(smooth(target))



color = color_sensor.color()



# Подтверждение синего

if color in abi:

blue_count += 1

else:

blue_count = 0



# Подтверждение оранжевого

if color in narengi:

orange_count += 1

else:

orange_count = 0



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



wait(5) # Пауза для разгрузки процессора и синхронизации потоков



# ==================================================

# ЭТАП 2: ОСНОВНОЙ ЗАЕЗД

# ==================================================

print("Основной цикл")



while True:

motor_b.run(speed)



color = color_sensor.color()



if color in abi:

blue_count += 1

else:

blue_count = 0



if color in narengi:

orange_count += 1

else:

orange_count = 0



line_detected = (blue_count >= 3 or orange_count >= 3)



# Защитный интервал в 1000 градусов по одометрии

if motor_b.angle() > 1000 and line_detected:

line_counter += 1

motor_b.reset_angle(0)

print("Линия:", line_counter)



blue_count = 0

orange_count = 0



if line_counter == 11:

break



# ------------------------

# Держим левую стену

# ------------------------

if direction == "LEFT":

distance = get_left()

target = (distance - 28) * -2

amotor(smooth(target))



# ------------------------

# Держим правую стену

# ------------------------

elif direction == "RIGHT":

distance = get_right()

target = (distance - 28) * 2

amotor(smooth(target))


wait(5)



# ==================================================

# ЭТАП 3: ДОЕЗД ДО ФИНИША

# ==================================================

print("Финальный участок")



# В Pybricks используем StopWatch вместо завязки на циклы со sleep

timer = StopWatch()

timer.reset()



# Интенсивный доезд в течение 600 миллисекунд

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



# ==================================================

# ФИНИШ

# ==================================================

motor_b.stop()

motor_a.run_target(600, 0) # Возвращаем руль ровно

