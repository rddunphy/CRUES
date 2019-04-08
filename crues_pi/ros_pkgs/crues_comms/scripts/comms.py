import socket
import json
import SocketServer
import threading


def _to_json(host_ip, hostname, sender_name, topic, data):
    # receiver ip and hostname, our ip and hostname, classname
    sender_ip = socket.gethostbyname(socket.gethostname())
    header = [host_ip, hostname, topic, sender_ip, sender_name, type(data).__name__]
    content = [header, data]
    message = json.dumps(content, separators=(',', ':'))
    return message


def _from_json(packet, ip):
    message = json.loads(packet)
    try:
        target = message[0][0]
        if target == ip:
            return message[0][2], message[1]
        else:
            return -1
    except IndexError:
        return -2


class ThreadedTCPRequestHandler(SocketServer.BaseRequestHandler):
    def messageHandler(x):
        print x

    def handle(self):
        data = self.request.recv(1024)
        message = _from_json(data, ThreadedTCPRequestHandler.ip)
        ThreadedTCPRequestHandler.messageHandler(message)


class ThreadedTCPServer(SocketServer.ThreadingMixIn, SocketServer.TCPServer):
    pass


def send(hostname, ip, sender_name, topic, data):
    # data is python object to send
    port = 8001            # The same port as used by the server
    message = _to_json(ip, hostname, sender_name, topic, data)
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((ip, port))
    try:
        sock.sendall(message)
    finally:
        sock.close()


def listen(handler, ip):
    ThreadedTCPRequestHandler.messageHandler = handler
    ThreadedTCPRequestHandler.ip = ip
    host, port = "0.0.0.0", 8001
    server = ThreadedTCPServer((host, port), ThreadedTCPRequestHandler)
    ip, port = server.server_address
    server_thread = threading.Thread(target=server.serve_forever)
    # Exit the server thread when the main thread terminates
    server_thread.daemon = True
    server_thread.start()
    return server


def kill_server(server):
    server.shutdown()
    server.server_close()
