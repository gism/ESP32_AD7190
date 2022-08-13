
#   Python Server to receive information from ESP32
#
#   Message should finish with '\n'
#   More information:
#   https://github.com/gism/ESP32_AD7190/


import socket

channel = ""
timestamp = 0
counter = 0
value = 0


def process_message(m):
    # print(m)
    a = m.split(" ")
    for data_package in a:
        d = data_package.split(":")
        if d[0] == 'n':
            channel = d[1]
        elif d[0] == 't':
            timestamp = d[1]
        elif d[0] == 'c':
            counter = d[1]
        elif d[0] == 'v':
            value = d[1]
        else:
            print("?" + d[0])
    line = "Channel: " + channel + " Counter: " + str(counter) + " Timestamp: " + str(timestamp) + " value: " + str(value)
    print(line)
    f.write(line + '\n')


f = open('log.txt', 'w')

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

s.bind(("0.0.0.0", 8000))
s.listen(3)

while True:

    client, addr = s.accept()
    print("Connected by {}".format(addr))
    message = ""

    while True:
        try:
            content = client.recv(200).decode("utf-8")

            for i in content:
                if i == '\n':
                    process_message(message)
                    message = ""
                else:
                    message = message + i

            if len(content) == 0:
                break

        except BlockingIOError:
            print("socket is open and reading from it would block")
        except ConnectionResetError:
            print("socket was closed for some other reason")
        except Exception as e:
            print("unexpected exception when checking if a socket is closed")

    print("Closing connection")
    client.close()
