# Gets data from our MongoDB (SBMDB) and sends its analytics to an MQTT Topic

from paho.mqtt.client import Client
from pymongo import MongoClient
from time import sleep

db = MongoClient('mongodb://mongodb:27017/')['SBMDB']

client = Client()
client.connect("broker.emqx.io")
client.loop_start()

while True:
	data = '' # ToDo: Get Data from MongoDB SBMDB and send analytics to topic
	client.publish("nick/iot/project/sbm/datasend", str(data))
	sleep(20.0)