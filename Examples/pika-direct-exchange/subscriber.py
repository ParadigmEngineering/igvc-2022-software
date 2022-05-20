""" subscriber.py

Receive messages from an AMQP direct exchange.
"""

import pika


if __name__ == "__main__":
    connection = pika.BlockingConnection(pika.ConnectionParameters(host="localhost"))
    channel = connection.channel()
    channel.exchange_declare("direct_logs", exchange_type="direct")

    result = channel.queue_declare(queue='A', exclusive=True)
    queue_name = result.method.queue
    channel.queue_bind(
        exchange="direct_logs",
        queue=queue_name,
        routing_key="A"
    )

    result = channel.queue_declare(queue='B', exclusive=True)
    queue_name = result.method.queue
    channel.queue_bind(
        exchange="direct_logs",
        queue=queue_name,
        routing_key="B"
    )

    print(' [*] Waiting for logs. To exit press CTRL+C')


    def callback(ch, method, properties, body):
        print(f" [x] {method.routing_key}:{body}")

    channel.basic_consume(
        queue="A",
        on_message_callback=callback,
        auto_ack=True
    )

    channel.basic_consume(
        queue="B",
        on_message_callback=callback,
        auto_ack=True
    )

    channel.start_consuming()
