import socket
import threading
import SocketServer
import random
import time
from concurrent.futures import ThreadPoolExecutor 

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):

    def handle(self):
        time.sleep(random.randrange(0,2))
        data = self.request.recv(1024)
        time.sleep(random.randrange(0,2))
        cur_thread = threading.current_thread()
        response = "{}: {}".format(cur_thread.name, data)
        self.request.sendall(response)

class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass

def client(ip, port, message):
    time.sleep(random.randrange(0, 2))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    time.sleep(random.randrange(0, 2))
    sock.connect((ip, port))
    try:
        time.sleep(random.randrange(0,2))
        sock.sendall(message)
        time.sleep(random.randrange(0,2))
        response = sock.recv(1024)
        print "Received: {}".format(response)
    finally:
        sock.close()

if __name__ == "__main__":
    # Port 0 means to select an arbitrary unused port
    HOST, PORT = "localhost", 0

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)
    ip, port = server.server_address

    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    print "Server loop running in thread:", server_thread.name

    with ThreadPoolExecutor(100) as executor:
        for i in range(100):
            executor.submit(client, ip, port, "Hello World " + str(i))

    server.shutdown()
    server.server_close()