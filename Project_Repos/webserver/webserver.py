# Colin Li 
# server socket python code 
# access on localhost:6789/HelloWorld.html 

#import socket module 
from socket import * 
import sys # In order to terminate the program 
serverSocket = socket(AF_INET, SOCK_STREAM) 

#Prepare a server socket 
#Fill in start 
serverPort = 6789 #use example port :6789
#starts TCP server based on TCPServer code in 2.7 socket programming tutorial 
serverSocket.bind(('', serverPort))
serverSocket.listen(1)  #Number of connections listening; > 1 
print("The server is ready on port:", serverPort)
#Fill in end 

while True: 
    #Establish the connection 
    print('Ready to serve...') 
    connectionSocket, addr =  serverSocket.accept()  #Fill in -> 2.7 TCP ex 
    try: 
        message = connectionSocket.recv(1024)     #Fill in -> 2.7 TCP ex 
        filename = message.split()[1]
        f = open(filename[1:]) #f gets file content 
        outputdata = f.readlines() #Fill in py file read function
        # use .readlines not .read or misses <!doctype HTML>
        
        #Send one HTTP header line into socket 
        #Fill in start 
        connectionSocket.send("HTTP/1.1 200: OK\r\n".encode())
        #Fill in end  
        #Send the content of the requested file to the client 
        for i in range(0, len(outputdata)):
            connectionSocket.send(outputdata[i].encode()) 
            
        connectionSocket.send("\r\n".encode()) 
        connectionSocket.close() 
        
#404 error section 
    except IOError: 
        #Send response message for file not found 
        connectionSocket.send("HTTP/1.1 Error 404: File Not Found\r\n".encode()) #fill in 
        
        #Close client socket 
        connectionSocket.close() #fill in 

#close server 
serverSocket.close() 
sys.exit()#Terminate the program after sending the corresponding data