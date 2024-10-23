import sys
import os
sys.path.append(os.getcwd())
from coppeliasim_zmqremoteapi_client import *
import threading
import time

client_main = RemoteAPIClient()
sim = client_main.require('sim')
sim.startSimulation()

stop_thread1_event = threading.Event()
stop_thread2_event = threading.Event()

gDistance1 = 0
gDistance2 = 0

def thread_1():
    global gDistance1
    client1 = RemoteAPIClient()
    sim1 = client1.require('sim')
    sensorhandle = sim1.getObject('./ultrasonic_sensor1')
    while not stop_thread1_event.is_set():
        gDistance1 = sim1.readProximitySensor(sensorhandle)[1]
        time.sleep(0.1)

def thread_2():
    global gDistance2
    client2 = RemoteAPIClient()
    sim2 = client2.require('sim')
    sensorhandle = sim2.getObject('./ultrasonic_sensor2')
    while not stop_thread1_event.is_set():
        gDistance2 = sim2.readProximitySensor(sensorhandle)[1]
        time.sleep(0.1)

thread1 = threading.Thread(target=thread_1)
thread2 = threading.Thread(target=thread_2)
thread1.start()
thread2.start()

start_time = time.time()
while time.time() - start_time < 10:
    print(f'Distance1: {gDistance1} Distance2: {gDistance2}')
    time.sleep(0.1)

stop_thread1_event.set()
stop_thread2_event.set()
thread1.join()
thread2.join()

sim.stopSimulation()
