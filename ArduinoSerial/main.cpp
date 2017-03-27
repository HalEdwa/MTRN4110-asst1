#include <winSock.h>
#include <windows.h> 
#include <iostream> 
#include <conio.h>
#include <thread>
#include <string>
#include "Serial.h"



#define MY_MESSAGE_NOTIFICATION      1048 //Custom notification message
HWND hwnd;
SOCKET ss; //Server
SOCKET ClientSock; //Client ID on Server Side

#define DEFAULT_BUFLEN 512

char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;



void CloseConnection(SOCKET ks) {
	if (ks) {
		closesocket(ks);
	}
	WSACleanup();
}

int ListenOnPort(int portno)
{

	WSADATA w;
	int error = WSAStartup(0x0202, &w);   // Fill in WSA info

	if (error)
	{
		std::cout << "winsock error" << std::endl;
		return false; //For some reason we couldn't start Winsock

	}

	if (w.wVersion != 0x0202) //Wrong Winsock version?
	{
		WSACleanup();
		std::cout << "wrong version of winsock" << std::endl;
		return false;
	}

	SOCKADDR_IN addr; // The address structure for a TCP socket

	addr.sin_family = AF_INET;      // Address family
	addr.sin_port = htons(portno);   // Assign port to this socket

									 //Accept a connection from any IP using INADDR_ANY
									 //You could pass inet_addr("0.0.0.0") instead to accomplish the 
									 //same thing. If you want only to watch for a connection from a 
									 //specific IP, specify that //instead.
	addr.sin_addr.s_addr = htonl(INADDR_ANY);

	ss = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); // Create socket

	if (ss == INVALID_SOCKET)
	{
		std::cout << "invalid socket" << std::endl;
		return false; //Don't continue if we couldn't create a //socket!!
	}
	//u_long iMode = 1;
	//ioctlsocket(ss, FIONBIO, &iMode);

	if (bind(ss, (LPSOCKADDR)&addr, sizeof(addr)) == SOCKET_ERROR)
	{
		//We couldn't bind (this will happen if you try to bind to the same  
		//socket more than once)
		std::cout << "bind" << std::endl;
		return false;
	}

	//Now we can start listening (allowing as many connections as possible to  
	//be made at the same time using SOMAXCONN). You could specify any 
	//integer value equal to or lesser than SOMAXCONN instead for custom 
	//purposes). The function will not //return until a connection request is 
	//made
	return listen(ss, SOMAXCONN);


	//Don't forget to clean up with CloseConnection()!
}

void acceptThread() {
	//CloseConnection(ClientSock);
	ClientSock = INVALID_SOCKET;
	ClientSock = accept(ss, NULL, NULL);
}


int main() {

	Serial SR("\\\\.\\COM3");
	if (SR.IsConnected() == 0) {
		std::cout << "Did not Connect Serial" << std::endl;
		Sleep(10000);
		return 1;
	}
	

	std::cout << "Press any key to start the Server:" << std::endl;
	if (_getch()) {
		if (ListenOnPort(15000) != 0) { //CHOSE THE SERVER PORT HERE!
			std::cout << "listen on port failed" << std::endl;
		}
		else {
			std::cout << "Server started" << std::endl;
		}
	}

	std::thread Accept(acceptThread);
	Accept.join();
	std::cout << "connection joined" << std::endl;

	if (ClientSock == INVALID_SOCKET) {
		std::cout << "Bad Socket" << std::endl;
	}

	char incomingData[13] = "A000B000C000";
	while (1) {
		
		SR.ReadData(incomingData, 12);
		std::cout <<"Pre-Shift:"<< incomingData << std::endl;
		while (incomingData[0] != 'A') {
			//char shift 
			char holder = incomingData[0];
			for (int i = 0; i < 11; i++) {
				incomingData[i] = incomingData[i + 1];
			}
			incomingData[11] = holder;

		}
		std::cout << "Pre-PostShift:" << incomingData << std::endl;
		send(ClientSock, incomingData, 12, 0);
		
		
		Sleep(1000);
	}

	return 0;
}