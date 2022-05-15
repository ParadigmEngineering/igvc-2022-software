"""  publisher.py

Generate and send messages to a direct exchange with 
alternating routing keys. 
"""


import pika
import time


if __name__=="__main__":
    connection = pika.BlockingConnection(pika.ConnectionParameters(host="localhost"))
    channel = connection.channel()
    channel.exchange_declare(
        exchange="direct_logs",
        exchange_type="direct"
    )

    try:
        while 1:
            # Publish with routing key A 
            message = "Message A!"
            channel.basic_publish(
                exchange="direct_logs",
                routing_key="A",
                body=message
            )
            print("Sent message on route A")
            time.sleep(2)

            # Publish with routing key B
            message = "Message B!"
            channel.basic_publish(
                exchange="direct_logs",
                routing_key="B",
                body=message
            )
            print("Sent message on route B")
            time.sleep(2)

    except KeyboardInterrupt: 
            print("Keyboard interrupt! Exiting...")
            connection.close()
