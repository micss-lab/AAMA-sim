import pika
import sys
import time
import uuid
import json
import threading

USERNAME = 'dogu'
PASSWORD = 'dogu'


class RabbitCommunication:
    def __init__(self, host='127.0.0.1', port=5672):
        self.host = host
        self.port = port
        self.connect()
        self.queue_callbacks = {}
        self.request_callbacks = {}
        self.response = None
        self.corr_id = None
        self.request_queues_initialized = False

    def connect(self):
        credentials = pika.PlainCredentials(USERNAME, PASSWORD)
        self.connection = pika.BlockingConnection(
            pika.ConnectionParameters(host=self.host, port=self.port, credentials=credentials, heartbeat=0))
        self.channel = self.connection.channel()
        self.channel.basic_qos(prefetch_count=1)

    def init_request_queues(self):
        if not self.request_queues_initialized:
            queue_declaration = self.channel.queue_declare(queue='', auto_delete=True)
            self.response_queue = queue_declaration.method.queue
            self.channel.basic_consume(queue=self.response_queue,
                                       on_message_callback=self.on_response,
                                       auto_ack=True)
            self.request_queues_initialized = True

    def declare_queue(self, queue_name):
        try:
            self.channel.queue_declare(queue=queue_name,
                                       auto_delete=True,
                                       arguments={"x-max-length": 1,
                                                  "x-overflow": "drop-head"})
        except pika.exceptions.ChannelClosedByBroker as ex:
            # make sure that if we are redefining one of the queues,
            # we reconnect to the channel and delete the previous queue definition
            # also redefine the queue
            print('ERROR:queue declaration error! queue_name:' + queue_name)
            print('Reconnecting the channel')
            self.connect()
            # delete the queue for definition
            self.channel.queue_delete(queue_name)
            self.declare_queue(queue_name)
        return True

    def register_to_queue(self, queue_name, user_callback):
        if self.declare_queue(queue_name):
            self.channel.basic_consume(queue=queue_name,
                                       on_message_callback=self.queue_callback,
                                       auto_ack=True)
            self.queue_callbacks[queue_name] = user_callback

    def queue_callback(self, ch, method, props, body):
        # if type(body) is str:
        #     try:
        #         body = json.loads(body)
        #     except:
        #         pass
        try:
            body = json.loads(body)
        except:
            pass

        self.queue_callbacks[method.routing_key](body)

    def register_queue(self, queue_name):
        self.declare_queue(queue_name)

    def send(self, queue_name, data):
        if type(data) is not str:
            data = json.dumps(data)

        self.channel.basic_publish(exchange='',
                                   routing_key=queue_name,
                                   body=data)

    def register_request(self, request_name, request_callback):
        self.init_request_queues()
        self.channel.queue_declare(queue=request_name)
        self.channel.basic_consume(queue=request_name, on_message_callback=self.on_request)
        self.request_callbacks[request_name] = request_callback

    def on_request(self, ch, method, props, body):
        response = self.request_callbacks[method.routing_key](body)
        if type(response) is not str:
            response = json.dumps(response)

        new_props = pika.BasicProperties(correlation_id=props.correlation_id)
        ch.basic_publish(exchange='',
                         routing_key=props.reply_to,
                         properties=new_props,
                         body=response)
        ch.basic_ack(delivery_tag=method.delivery_tag)

    def is_request_registered(self, request_name):
        try:
            queue_declaration = self.channel.queue_declare(queue=request_name)
        except ValueError as ex:
            return False
        if queue_declaration.method.consumer_count > 0:
            return True
        else:
            self.channel.queue_delete(request_name)
            return False

    def send_request(self, request_name, params=''):
        self.init_request_queues()
        if not self.is_request_registered(request_name):
            print("ERROR: Not registered request:" + request_name)
            return None
        if type(params) is not str:
            params = json.dumps(params)
        self.response = None
        self.corr_id = str(uuid.uuid4())
        props = pika.BasicProperties(reply_to=self.response_queue, correlation_id=self.corr_id)
        self.channel.basic_publish(exchange='',
                                   routing_key=request_name,
                                   properties=props,
                                   body=params)
        while self.response is None:
            self.connection.process_data_events()
            time.sleep(0.0001)
        return self.response

    def on_response(self, ch, method, props, body):
        if self.corr_id == props.correlation_id:
            if type(body) is str:
                try:
                    body = json.loads(body)
                except:
                    pass
            self.response = body

    def start_listening(self):
        self.channel.start_consuming()


def callback0_test(data):
    print("test_callback0 data: %s" % data)


def callback1_test(data):
    print("test_callback1 data: %s" % data)


def consumer_producer_test():
    rabbit_comm = RabbitCommunication()
    if sys.argv[1] == 'listen':
        rabbit_comm.register_to_queue(sys.argv[2], callback0_test)
        rabbit_comm.register_to_queue(sys.argv[3], callback1_test)
        rabbit_comm.start_listening()
    elif sys.argv[1] == 'send':
        rabbit_comm.register_queue(sys.argv[2])
        while True:
            msg = input()
            rabbit_comm.send(sys.argv[2], msg)


def request0(param):
    return 'request0: ' + param + ' ' + param


def request1(param):
    return 'request1: ' + param + ' ' + param


def request_response_test():
    rabbit_comm = RabbitCommunication()
    if sys.argv[1] == 'request':
        rabbit_comm.register_request('request0', request0)
        rabbit_comm.register_request('request1', request1)
        rabbit_comm.start_listening()
    elif sys.argv[1] == 'response':
        count = 0
        while count < 100:
            print(rabbit_comm.send_request('request0', 'sending request ' + str(count)))
            count += 1
            print(rabbit_comm.send_request('request3', 'sending request ' + str(count)))
            count += 1
            print(rabbit_comm.send_request('request1', 'sending request ' + str(count)))
            count += 1


def param_callback(request_param):
    params = dict()
    params['uav_count'] = 1
    params['max_casualty'] = 10
    return params


def uav_pose_callback(pose):
    print('uav_pose callback:' + str(pose))


def cmd_callback(cmd):
    print('cmd:' + str(cmd))


def consumer_producer_with_threads_test():
    if sys.argv[1] == 'team':
        # listenning for uav pose
        uav_pose_queue = 'uav_pose'
        listen_comm = RabbitCommunication()
        listen_comm.register_to_queue(uav_pose_queue, uav_pose_callback)
        callback_thread = threading.Thread(target=listen_comm.start_listening)
        callback_thread.start()

        rabbit_comm = RabbitCommunication()
        # get scenario parameters
        params = rabbit_comm.send_request('parameters')
        print('params:')
        print(params)

        # uav command sending
        cmd_queue = 'uav_command'
        rabbit_comm.register_queue(cmd_queue)
        count = 0
        while True:
            command = [0, 1, count]
            rabbit_comm.send(cmd_queue, command)
            count += 1
            time.sleep(0.01)
    elif sys.argv[1] == 'ygk':
        # listening parameters and uav_command
        listen_comm = RabbitCommunication()
        listen_comm.register_request('parameters', param_callback)
        cmd_queue = 'uav_command'
        listen_comm.register_to_queue(cmd_queue, cmd_callback)
        cmd_thread = threading.Thread(target=listen_comm.start_listening)
        cmd_thread.start()

        # send uav pose messages
        pose_comm = RabbitCommunication()
        uav_pose_queue = 'uav_pose'
        pose_comm.register_queue(uav_pose_queue)
        count = 0
        while True:
            pose = [count, count + 1, count + 2]
            count += 1
            pose_comm.send(uav_pose_queue, pose)
            time.sleep(0.01)

# if __name__ == "__main__":
# consumer_producer_test()
# request_response_test()
# consumer_producer_with_threads_test()
