from comms import send
import threading
import time
import random


class MyThread(threading.Thread):
	def run(self):
		time.sleep(random.uniform(1.0, 5.0))
		send("blinky", "192.168.1.2", "inky", "test_topic", "Hello from {}".format(self.getName()))
		
		
def main():
	i = 1
	while i < 100:
		t = MyThread(name="Thread {}".format(str(i)))
		t.start()
		i += 1


if __name__ == '__main__':
	main()
