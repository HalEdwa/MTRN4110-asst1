#include <winSock.h>
#include <windows.h> 
#include <iostream> 
#include <conio.h>
#include <thread>
#include <string>
#include "Serial.h"
#include <chrono>

#define MY_MESSAGE_NOTIFICATION 1048 //Custom notification message
#define BUFFERSIZE 17

HWND hwnd;
SOCKET ss; //Server
SOCKET ClientSock; //Client ID on Server Side

auto system_start = std::chrono::high_resolution_clock::now();
auto lastTime = std::chrono::high_resolution_clock::now();
auto currentTime = std::chrono::high_resolution_clock::now();
auto dur = currentTime - lastTime;

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

void synchronise(Serial sr) {
	char* read_data = new char[BUFFERSIZE];
	
	std::cout << "Need to Synchronise. Searching for 'HD'" << std::endl;
	bool synchronised = FALSE;
	
	while (!synchronised) {
		sr.ReadData(read_data, 16);
		
		/*
		while (read_data[0] != 'H') {	// Search for H
			sr.ReadData(&read_data[0], 1);
			std::cout << "Search H: " << (int*)read_data << std::endl;
		}

		sr.ReadData(&read_data[0], 1);
		std::cout << "Search D: " << (int*)read_data << std::endl;
		std::cout << '\n' << '\n' << std::endl;

		if (read_data[1] == 'D') {
			synchronised = TRUE;
		}
		*/
		std::cout << "Start thang" << std::endl;
		int val = ((read_data[0] & 0xff) << 8) | (read_data[1] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[2] & 0xff) << 8) | (read_data[3] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[4] & 0xff) << 8) | (read_data[5] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[6] & 0xff) << 8) | (read_data[7] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[8] & 0xff) << 8) | (read_data[9] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[10] & 0xff) << 8) | (read_data[11] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[12] & 0xff) << 8) | (read_data[13] & 0xff);
		std::cout << val << std::endl;
		val = ((read_data[14] & 0xff) << 8) | (read_data[15] & 0xff);
		std::cout << val << '\n' << std::endl;
		Sleep(100);
	}

	std::cout << '\n' << "Found 'HD'. Offsetting" << std::endl;
	sr.ReadData(&read_data[0], BUFFERSIZE - 3);
	std::cout << "Synchronised Buffer: " << read_data << std::endl;
}

int main() {

	Serial SR("\\\\.\\COM6");

	if (SR.IsConnected() == 0) {
		std::cout << "Did not Connect Serial" << std::endl;
		Sleep(2000);
		return 1;
	}
	
	std::cout << "Press any key to start the Server:" << std::endl;
	if (_getch()) {
		if (ListenOnPort(15000) != 0) { 
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
	
	char* incomingData = new char[BUFFERSIZE];

	synchronise(SR);

	while (1) {
		int freq = 2;
		int counter = 0;
		currentTime = std::chrono::high_resolution_clock::now();
		dur = currentTime - lastTime;

		//auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
		//std::cout << "Time Elapsed: " << ms << std::endl;

		// Check time elapsed
		if ((currentTime - lastTime) > (std::chrono::milliseconds::duration(1000 / freq))) {
			lastTime = currentTime;
			SR.ReadData(incomingData, BUFFERSIZE - 1);

			if (incomingData[0] == 72 && incomingData[1] == 68) {	//search for H
				// Send data over TCP
				send(ClientSock, incomingData, BUFFERSIZE - 1, 0);
				std::cout << incomingData << std::endl;
			}
			else {
				synchronise(SR);
			}
		}
	}

	return 0;
}