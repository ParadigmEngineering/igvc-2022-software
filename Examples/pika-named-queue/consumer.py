""" consumer.py

Consume and log messages from a RabbitMQ AMQP message queue. 
"""

import pika
import sys
import os


def main():
    connection = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = connection.channel()

    # Ensure queue exists, not necessary if we are sure queue exists,
    # Good practice though
    channel.queue_declare("hello")

    # Define and register callback
    def callback(ch, method, properties, body):
        print(f"[x] Received message: {body}")
        return

    channel.basic_consume(
        queue="hello",
        auto_ack=True,
        on_message_callback=callback
    )

    print(' [*] Waiting for messages. To exit press CTRL+C')
    channel.start_consuming()

if __name__=="__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Interrupted...")
        try:
            sys.exit(0)
        except SystemExit:
            os._exit(0)
