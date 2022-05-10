# RabbitMQ Message Broker 
RabbitMQ is an open source message broker application that implements the Advanced Message Queuing Protocl (AMQP). We will use rabbitmq to implement a publish/subscribe model in the `Canstation` tool. 

## Daemon - Docker 
The easiest way to launch the rabbitmq message broker is using docker. At some point, instructions will also be included for installing and managing rabbitmq without using docker. 

An important note. When using the RabbitMQ docker container, the host name parameter (`-h`, `--hostname`) should be specified. This name is used to identify the location where RMQ will store data. 
```
 docker run -d -h my-rabbit -p 5672:5672 --name some-rabbit rabbitmq:3
```

This shoud launch an RMQ container running ond efault port 5672. Then, confirm operation with:
```
docker logs some-rabbit
```

## RabbitMQ Commands
List existing queues

Linux 
```
sudo rabbitmqctl list_queues 
```
Windows
```
rabbitmqctl.bat list_queues
```
