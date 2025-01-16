# Gets data from an MQTT Topic and saves it to our MongoDB (SBMDB)

from paho.mqtt.client import Client
from pymongo import MongoClient

db = MongoClient('mongodb://mongodb:27017/')["SBMDB"]

def on_connect(client, userdata, flags, rc):
	client.subscribe("nick/iot/project/sbm/datamine")

def on_message(client, userdata, msg):
	db["data"].insert_one({"value": msg.payload.decode("utf-8")})

client = Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("broker.emqx.io", 1883, 120)
client.loop_forever()