# Client Program
import socket
import json
from paths import config_file
import SocketServer
import threading
import time


CONFIG_FILE_NAME = "ip_addresses.json"


def load_addresses():
    path = config_file(CONFIG_FILE_NAME)
    with open(path) as f:
        return json.load(f)


addresses = load_addresses()


def lookupip(name):
    return addresses[name]


def lookupname(search_ip):
    for name, ip in addresses.items():
        if ip == search_ip:
            return name
    return "IP Not Found"       # if IP can't be found



def tojson(host_ip, hostname, time_stamp, data):
    # receiver ip and hostname, our ip and hostname, classname
    sender_ip = socket.gethostbyname(socket.gethostname())
    header = [host_ip, hostname, sender_ip, lookupname(sender_ip), time_stamp, type(data).__name__]
    content = [header, data]
    message = json.dumps(content, separators=(',', ':'))
    return message


def fromjson(packet):
    message = json.loads(packet)
    try:
        target = message[0][0]
        #print target
        #print socket.gethostname()
        #print Address_lookup.lookupip(socket.gethostname())
        if target == lookupip(socket.gethostname()):
            return message
        else:
            return -1
    except IndexError:
        return -2

# End of Helper Functions

class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    pub = None

    def handle(self):
        data = self.request.recv(1024)
        message = fromjson(data)
        ThreadedTCPRequestHandler.pub.publish(message)


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass
    
#End of Class Definitions
    
    
    
    
# data is python object to send
def send(hostname, data):
    host_ip = lookupip(hostname)    # The remote host
    #print host_ip
    port = 8001            # The same port as used by the server

    time_stamp = time.time()
    message = tojson(host_ip, hostname, time_stamp, data)
    #print message
    #print fromjson(message)
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    #if not s.connect_ex((host_ip, port_number)):
    #    print s.getsockname()
    #    #print "connected"
    #    s.sendall(message)          # Sends string (of JSON)
        # data = s.recv(1024)       # used as ACK so uncomment if needed
    #    s.close()
        
        
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host_ip, port))
    try:
        sock.sendall(message)
    finally:
        sock.close()




def listen():#pub):
    #ThreadedTCPRequestHandler.pub = pub
    ThreadedTCPRequestHandler.handle = dump_log
    HOST, PORT = "0.0.0.0", 8001

    server = ThreadedTCPServer((HOST, PORT), ThreadedTCPRequestHandler)

    ip, port = server.server_address
    #ThreadedTCPRequestHandler.set_pub(pub)
    # Start a thread with the server -- that thread will then start one
    # more thread for each request
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    return server


def killServer(server):
    server.shutdown()
    server.server_close()
    """
    # host_ip = socket.gethostbyname(socket.gethostname())  # Symbolic name meaning all available interfaces
    PORT = 8001  # Arbitrary non-privileged port
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(('0.0.0.0', PORT))
    s.listen(2)
    conn, addr = s.accept()
    #print 'Connected by', addr
    packets = '' #"ERRORVALUE"
    while 1:
        packet = conn.recv(1024)
        if not packet:
            break
        packets += packet
   # conn.sendall("ACK")
    conn.close()
    #print "packet: {" + packets + "}"
    message = fromjson(packets)
    #print "Message received"
    #print message
    return message"""


import sys

output = []


def dump_log(self):
    data = self.request.recv(1024)
    message = fromjson(data)
    received = time.time()
    latency = received - message[0][4]
    global output
    output += [(sys.getsizeof(message), latency)]

import os


def send_bytes(n):
    send("t430", os.urandom(n))
