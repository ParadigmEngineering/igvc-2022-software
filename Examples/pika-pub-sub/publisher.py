""" publisher.py

Broadcast messages to a RabbitMQ 
message queue with a fanout exchange. 
"""

import pika 


if __name__ == "__main__":
    connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
    channel = connection.channel()

    # Crete fanout exchange called logs
    channel.exchange_declare(
        exchange="logs",
        exchange_type="fanout"
    )

    # Publish message using fanout exchange
    # Still need routing key but its value is ignored for fanout
    message = "Yo Dawg"
    channel.basic_publish(
        exchange="logs",
        routing_key="",
        body=message
    )

    print(f"Sent message: {message}")
    connection.close()
