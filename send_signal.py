from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil 
import time
import math

def send_signal (vehicle, param = [0, 0, 0.6, 0]):
#параметры (дрон, крен, тангаж, тяга, рысканье)
    criticalangles=10
    criticalthrottle=0.6
 
#Каналы: 1=Roll крен, 2=Pitch тангаж , 3=Throttle, 4=Yaw рысканье
#Проверка критических углов, если не проходит, устанавливаем их
 
    if param[1]>criticalangles:
        pitch=criticalangles
    else:
        pitch=param[1]
          
    if param[3]>criticalangles:
        yaw=criticalangles
    else:
        yaw=param[3]
        
    if param[0]>criticalangles:
        roll=criticalangles 
    else:
        roll=param[0]

    if param[2]<criticalangles:
        throttle=criticalthrottle
    else:
        throttle=param[2]

#Установка новых углов (прием в градусах)
    
    vehicle.channels.overrides = {'1':degrees(roll), '2':degrees(pitch),'3':degrees(throttle), '4':degrees(yaw)}  
#Проверка, что углы установились
    return(check_signal(vehicle, [roll, pitch,throttle,yaw]))

def check_signal(vehicle, param = [0, 0, 0.6, 0]):
    differece=0.1
#Проверка успешности вычитаем что есть из того, что задавали и сравниваем с ошибкой 
    check1 = all([vehicle.channels['1']-param[0]<differece,
                  vehicle.channels['2']-param[1]<differece,
                  vehicle.channels['4']-param[3]<differece])
    check2 = not any([vehicle.channels['1']-param[0]<differece,
                      vehicle.channels['2']-param[1]<differece,
                      vehicle.channels['4']-param[3]<differece])
#проверку через flush?
    if (check1):
        return(0)
    elif (check2):
        return(1)
    else:
        return(2)
     
def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
#Рысканье управляется либо через угол, либо скорость, автопилот использует только что-то одно
#Тяга от 0 до 1

    if yaw_angle is None:
        # если угла рысканья нет, то получаем
        yaw_angle = vehicle.attitude.yaw
        # тяга зависания коптера =0.6
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # время с загрузки
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # кватернион из полученных данных
        0, # скорость круна в радианах/с
        0, # скорость тангажа в радианах/с
        math.radians(yaw_rate), # скорость рысканья в рад/с
        thrust  # тяга
    )
    vehicle.send_mavlink(msg)
    
def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
#переводим углы в кватернион
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]
