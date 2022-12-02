import pika
import json

credentials = pika.PlainCredentials('dogu', 'dogu')
connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost', port=5672,credentials=credentials))
channel = connection.channel()


message = {
    'x': -0.1,
    'y': 0
}

channel.basic_publish(exchange='', routing_key='tb3_1_ctrl', body=json.dumps(message))
print(" [x] Sent 'Hello World!'")
connection.close()