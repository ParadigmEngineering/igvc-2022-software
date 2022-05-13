""" consumer.py

Subscribe to a topic and log messages from a RabbitMQ AMQP 
message queue with a fanout exchange. 
"""
import pika 
import sys


if __name__ == "__main__":
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
    channel = connection.channel()
    
    # Good practice, exchange may not exist yet 
    channel.exchange_declare("logs", exchange_type="fanout")

    result = channel.queue_declare('', exclusive=True)
    queue_name = result.method.queue

    channel.queue_bind(exchange="logs", queue=queue_name)

    print(' [*] Waiting for logs. To exit press CTRL+C')

    def callback(ch, method, properties, body):
        print(f" [x] {body}")

    channel.basic_consume(
        queue=queue_name, 
        on_message_callback=callback, 
        auto_ack=True
    )

    channel.start_consuming()
