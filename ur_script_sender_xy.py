import socket


def toBytes(str):
    return bytes(str.encode())


with open("move_ee_xy.script", "r") as file:
    prog = file.read()

HOST = "192.168.0.135"
PORT = 30001

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
s.send(toBytes(prog))
