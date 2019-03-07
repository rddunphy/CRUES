# Client Program
import socket
import json


addresses = {'Lagann' : '192.168.1.1','blinky': '192.168.1.2', 'inky': '192.168.1.3', 'clyde': '192.168.1.4'}


def lookupip(name):
    return addresses[name]


def lookupname(search_ip):
    for name, ip in addresses.items():
        if ip == search_ip:
            return name
    return "IP Not Found"       # if IP can't be found



def tojson(host_ip, hostname, data):
    # receiver ip and hostname, our ip and hostname, classname
    sender_ip = socket.gethostbyname(socket.gethostname())
    header = [host_ip, hostname, sender_ip, lookupname(sender_ip), type(data).__name__]
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
            return message[1]
        else:
            return -1
    except IndexError:
        return -2


# data is python object to send
def send(hostname, data):
    host_ip = lookupip(hostname)    # The remote host
    #print host_ip
    port_number = 8001              # The same port as used by the server
    message = tojson(host_ip, hostname, data)
    #print message
    #print fromjson(message)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    if not s.connect_ex((host_ip, port_number)):
        #print "connected"
        s.sendall(message)          # Sends string (of JSON)
        # data = s.recv(1024)       # used as ACK so uncomment if needed
        s.close()




def listen():
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
    return message
