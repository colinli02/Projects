# UDPPingerClient.py
# Colin Li

# Client Program Functions:
# Send 10 ping messages, wait 1 second between each ping (timeout 1s)  
# (1) send the ping message using UDP 
#     (Note: Unlike TCP, you do not need to establish a connection first, 
#     since UDP is a connectionless protocol.)
# (2) print the response message from server, if any
# (3) calculate and print the round trip time (RTT), in seconds, of each packet, if server responses
# (4) otherwise, print “Request timed out”

from time import * 
from socket import *

# Create client socket 
serverName = 'localhost'
serverPort = 12000
clientSocket = socket(AF_INET, SOCK_DGRAM)

# Starts our number of pings at 1 
numOfPings = 1 

# "Python documentation to find out how to set the timeout value on a datagram socket"
# Sets timeout at 1 s 
clientSocket.settimeout(1)

#for(i = 0; i<11; i++) 
for numOfPings in range(10):
    start = time()
    
    #(1) Send the ping message using UDP 
    # Ping sequence_number time 
    endPrime = time()
    timeElapsedPrime = endPrime - start
    message = "Ping: %s %s" % ( numOfPings, timeElapsedPrime) #make sure is string, not tuple
    clientSocket.sendto(message.encode(),(serverName, serverPort))
  
    try:
    #(2) Prints response from server, if any and also-
        # Receives server response 
        modifiedMessage, serverAddress = clientSocket.recvfrom(2048)
    #(3) calculate RTT 
        end = time()
        timeElapsed = end - start 
        # PING, Sequence number (numOfPing+1), Time (RTT) 
        print(modifiedMessage, numOfPings+1,  "RTT:", timeElapsed*1000.0," ms")
    
    #(4) otherwise: rq timed out 
    # note to self: will always jump to except if compile error (timeElasped -> Elapsed, typo)
    except:
        print("Request timed out")
    
# Closes socket 
print("Closing socket port\n\n")
clientSocket.close()