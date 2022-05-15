""" producer.py

Produce a message to be consumed by consumers. 
"""

import pika


if __name__ == "__main__":
    # Connect to broker on localhost
    # To connect to a broker on a different machine, simply specify a different host here
    conn = pika.BlockingConnection(pika.ConnectionParameters('localhost'))
    channel = conn.channel()

    # Ensure queue exists 
    channel.queue_declare(queue="hello")

    # Publish message (in the most basic way possible)
    channel.basic_publish(
        exchange="",            # Use the default, basic exchange 
        routing_key="hello",    # Specify the queue
        body="Hello World!"
    )

    # Flush network buffers
    conn.close()
