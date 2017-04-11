#include <winsock2.h> 
#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <chrono>

/*Socket headers =======================================*/
#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include <ws2tcpip.h>
#include <stdlib.h>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")

/*Socket Define and Global====================*/
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "15000"

WSADATA wsaData;
int iResult;

SOCKET ListenSocket = INVALID_SOCKET;
SOCKET ClientSocket = INVALID_SOCKET;

struct addrinfo *result = NULL;
struct addrinfo hints;

int iSendResult;
char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;

auto system_start = std::chrono::high_resolution_clock::now();
auto lastTime = std::chrono::high_resolution_clock::now();

/*===============================================*/

int TCP_ServerConfig() {
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Create a SOCKET for connecting to server
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	
	if (ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return 1;
	}

	// Setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	freeaddrinfo(result);
	printf("Server is online\n");

	iResult = listen(ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	printf("Waiting for client to connect...\n");

	// Accept a client socket
	ClientSocket = accept(ListenSocket, NULL, NULL);
	if (ClientSocket == INVALID_SOCKET) {
		printf("accept failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	printf("Connetion established\n");

	// No longer need server socket
	closesocket(ListenSocket);
}

// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
	Serial* SP = new Serial("COM6");    // adjust as needed

	if (SP->IsConnected())
		printf("We're connected");

	//
	char incomingData[16] = "";			
	int dataLength = 16;
	int readResult = 0;

	TCP_ServerConfig();
	int counter = 0;
	int freq = 50;

	while (SP->IsConnected())
	{
		auto currentTime = std::chrono::high_resolution_clock::now();
		auto dur = currentTime - lastTime;

		if ((currentTime - lastTime) > (std::chrono::milliseconds::duration(1000 / freq))) {
			lastTime = currentTime;

			readResult = SP->ReadData(incomingData, dataLength);

			//printf("Bytes read: (0 means no data available) %i\n", readResult);
			//incomingData[readResult] = 0;

			counter++;
			//printf("%s \n", incomingData);

			/*====== Send buffer ======*/
			if (readResult == dataLength) {
				//printf("%s \n", (char*)incomingData);
				iSendResult = send(ClientSocket, (char*)incomingData, sizeof(incomingData), 0);
				if (iSendResult == SOCKET_ERROR) {
					printf("send failed with error: %d\n", WSAGetLastError());
					closesocket(ClientSocket);
					WSACleanup();
				}
				//printf("Bytes sent: %d\n", iSendResult);
				counter = 0;
			}
		}
	}
	return 0;
}
