import dbus
import time
import paho.mqtt.client as paho
from paho import mqtt
import threading
import json
import os

class ServiceManager:
    def __init__(self, prefix, user_mode=False):
        self.prefix = prefix
        # Connect to the session bus for user services if user_mode is True, otherwise use the system bus
        self.bus = dbus.SessionBus() if user_mode else dbus.SystemBus()
        self.systemd = self.bus.get_object('org.freedesktop.systemd1', '/org/freedesktop/systemd1')
        self.manager = dbus.Interface(self.systemd, 'org.freedesktop.systemd1.Manager')

    def get_service_status(self):
        statuses = []
        try:
            units = self.manager.ListUnits()
            for unit in units:
                name, desc, load_state, active_state, sub_state = unit[:5]
                if name.startswith(self.prefix) and name.endswith(".service"):
                    statuses.append({"service": name, "status": f"{active_state}", "substatus": f"{sub_state}"})
        except dbus.DBusException as e:
            print(f"Error getting service status: {e}")
        return statuses

    def control_service(self, service_name, action):
        try:
            # Call appropriate method on the manager interface
            if action == "Start":
                self.manager.StartUnit(service_name, "replace")
            elif action == "Stop":
                self.manager.StopUnit(service_name, "replace")
            elif action == "Restart":
                self.manager.RestartUnit(service_name, "replace")
            print(f"Service {service_name} {action} command sent.")
        except dbus.DBusException as e:
            print(f"Error controlling service {service_name}: {e}")

class MqttServiceMonitor:
    def __init__(self, broker, pub_topic, sub_topic, username, password, service_manager, interval):
        self.client = paho.Client(paho.CallbackAPIVersion.VERSION2)
        # enable TLS for secure connection
        self.client.tls_set(tls_version=mqtt.client.ssl.PROTOCOL_TLS)
        self.client.username_pw_set(username, password)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(broker, 8883)
        
        self.pub_topic = pub_topic
        self.sub_topic = sub_topic
        self.service_manager = service_manager
        self.interval = interval

        self.monitor_thread = threading.Thread(target=self.start_monitoring)
        self.monitor_thread.daemon = True

    def on_connect(self, client, userdata, flags, rc, properties=None):
        print(f"Connected to MQTT broker with result code {rc}")
        client.subscribe(self.sub_topic)

    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8')
        try:
            message = json.loads(payload)
            service_name = message.get("service")
            request_action = message.get("request_action")
            if service_name and request_action in ["Start", "Stop", "Restart"]:
                print(f"Service request received: {service_name} - {request_action}")
                self.service_manager.control_service(service_name, request_action)
        except json.JSONDecodeError:
            print("Received invalid JSON message")

    def publish_service_status(self):
        statuses = self.service_manager.get_service_status()
        for status in statuses:
            payload = json.dumps(status)  # Convert the status dict to JSON
            self.client.publish(self.pub_topic, payload)
            print(f"Published status: {payload}")

    def start_monitoring(self):
        while True:
            self.publish_service_status()
            time.sleep(self.interval)

    def start(self):
        self.monitor_thread.start()
        self.client.loop_forever()


def main():
    broker = os.environ.get("MQTT_BROKER")
    username = os.environ.get("MQTT_USERNAME")
    password = os.environ.get("MQTT_PASSWORD")
    robot_name = os.environ.get("ROBOT_NAME", "robot10")
    pub_topic = f"{robot_name}/services/status"
    sub_topic = f"{robot_name}/services/control"
    service_prefix = "bigbot" # e.g., "myservice_"
    interval = 10                         # Time interval in seconds
    user_mode = True                      # Set to True for user-defined services

    if not broker or not username or not password:
        raise RuntimeError("MQTT_BROKER, MQTT_USERNAME, and MQTT_PASSWORD environment variables must be set.")

    service_manager = ServiceManager(service_prefix, user_mode=user_mode)
    monitor = MqttServiceMonitor(broker, pub_topic, sub_topic, username, password, service_manager, interval)
    monitor.start()

if __name__ == "__main__":
    main()