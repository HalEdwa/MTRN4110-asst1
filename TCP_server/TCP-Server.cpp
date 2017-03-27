#undef UNICODE

#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

// Need to link with Ws2_32.lib
#pragma comment (lib, "Ws2_32.lib")
// #pragma comment (lib, "Mswsock.lib")

#define DEFAULT_BUFLEN 3
#define DEFAULT_PORT "27015"

int __cdecl main(void)
{
	WSADATA wsaData;
	int iResult;

	SOCKET ListenSocket = INVALID_SOCKET;
	SOCKET ClientSockets = INVALID_SOCKET;
	SOCKET ClientSocketr = INVALID_SOCKET;

	struct addrinfo *result = NULL;
	struct addrinfo hints;

	int iSendResult;
	char recvbuf[DEFAULT_BUFLEN];
	int recvbuflen = DEFAULT_BUFLEN;

	// Initialize Winsock
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

	iResult = listen(ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	// Accept a client socket
	ClientSockets = accept(ListenSocket, NULL, NULL);
	if (ClientSockets == INVALID_SOCKET) {
		printf("accept s failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}

	// Accept a client socket
	ClientSocketr = accept(ListenSocket, NULL, NULL);
	if (ClientSocketr == INVALID_SOCKET) {
		printf("accept r failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return 1;
	}


	// Receive until the peer shuts down the connection
	int freq = 25;
	unsigned short int measurements[10] = { 0 };
	int counter = 0;
	bool state = 1;
	recvbuf[0] = '1';
	recvbuf[1] = '2';
	recvbuf[2] = 0;

	do {
		recv(ClientSocketr, recvbuf, DEFAULT_BUFLEN, 0);

		std::cout << "Speed(Hz): " << recvbuf << std::endl;

		sscanf_s(recvbuf, "%d", &freq);

		if (freq == 0) break;

		if (freq > 50) freq = 50;

		Sleep(1000 / freq);

		if (measurements[0] == 0 || measurements[0] == 200) state = !state;

		for (int i = 0; i < 10; i++) {
			if (state) measurements[i]--;
			else measurements[i]++;

			std::cout << measurements[i] << '\t';
		}

		std::cout << '\n';

		send(ClientSockets, (char *)&measurements, 20, 0);


	} while (1);
	closesocket(ListenSocket);
	// shutdown the connection since we're done
	iResult = shutdown(ClientSockets, SD_SEND);
	if (iResult == SOCKET_ERROR) {
		printf("shutdown failed with error: %d\n", WSAGetLastError());
		closesocket(ClientSockets);
		WSACleanup();
		return 1;
	}

	// cleanup
	closesocket(ClientSockets);
	WSACleanup();

	return 0;
}
