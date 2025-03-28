from startup.start_docker_influxdb import start_docker_influxdb
from startup.start_docker_rabbitmq import start_docker_rabbitmq

if __name__ == '__main__':
    start_docker_rabbitmq()
    start_docker_influxdb()
