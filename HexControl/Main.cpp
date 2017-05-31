
#include <winsock2.h> 
#include <stdio.h>
#include <tchar.h>
#include "SerialClass.h"	// Library described above
#include <string>
#include <chrono>
#include <iostream>
#include <bitset>

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
#define DEFAULT_PORT "14000"

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
auto lastTime1 = std::chrono::high_resolution_clock::now();

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

	char incomingData[6] = "";
	char outgoingData[8] = "";
	int dataLength = 8;
	int readResult = 0;
	int ch = 300;

	TCP_ServerConfig();
	int counter = 0;
	int freq = 10;

	while (SP->IsConnected())
	{
		auto currentTime = std::chrono::high_resolution_clock::now();

		iSendResult = recv(ClientSocket, (char*)incomingData, sizeof(incomingData), 0);

		if (iSendResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
		}

		if ((currentTime - lastTime) > (std::chrono::milliseconds::duration(1000 / freq))) {
			lastTime = currentTime;

			/*====== Send buffer ======*/
			outgoingData[0] = 0xff;
			outgoingData[1] = incomingData[0];
			outgoingData[2] = incomingData[1];
			outgoingData[3] = incomingData[2];
			outgoingData[4] = incomingData[3];
			outgoingData[5] = incomingData[4];
			outgoingData[6] = incomingData[5];
			outgoingData[7] = 0xff - (0xff & (incomingData[0] + incomingData[1] + incomingData[2] + incomingData[3]));
			//memcpy(&outgoingData[1], &incomingData, 6 * sizeof(int8_t));

			printf("%d %d %d %d %d %d %d %d\n", (uint8_t)outgoingData[0], (uint8_t)outgoingData[1], (uint8_t)outgoingData[2], (uint8_t)outgoingData[3],
				(uint8_t)outgoingData[4], (uint8_t)outgoingData[5], (uint8_t)outgoingData[6], (uint8_t)outgoingData[7]);

			readResult = SP->WriteData(outgoingData, dataLength);
		}
	}
	return 0;
}