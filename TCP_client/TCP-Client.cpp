#define WIN32_LEAN_AND_MEAN

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream> 
#include <conio.h>

// Need to link with Ws2_32.lib, Mswsock.lib, and Advapi32.lib
#pragma comment (lib, "Ws2_32.lib")
#pragma comment (lib, "Mswsock.lib")
#pragma comment (lib, "AdvApi32.lib")


#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "27015"

int __cdecl main()
{
	WSADATA wsaData;
	SOCKET ConnectSocketr = INVALID_SOCKET;
	SOCKET ConnectSockets = INVALID_SOCKET;
	struct addrinfo *result = NULL,
		*ptr = NULL,
		hints;

	char recvbuf[DEFAULT_BUFLEN];
	int iResult;
	int recvbuflen = DEFAULT_BUFLEN;



	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return 1;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	// Resolve the server address and port
	iResult = getaddrinfo("127.0.0.1", DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return 1;
	}

	// Attempt to connect to an address until one succeeds
	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSocketr = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSocketr == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(ConnectSocketr, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSocketr);
			ConnectSocketr = INVALID_SOCKET;
			continue;
		}
		break;
	}

	for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

		// Create a SOCKET for connecting to server
		ConnectSockets = socket(ptr->ai_family, ptr->ai_socktype,
			ptr->ai_protocol);
		if (ConnectSockets == INVALID_SOCKET) {
			printf("socket failed with error: %ld\n", WSAGetLastError());
			WSACleanup();
			return 1;
		}

		// Connect to server.
		iResult = connect(ConnectSockets, ptr->ai_addr, (int)ptr->ai_addrlen);
		if (iResult == SOCKET_ERROR) {
			closesocket(ConnectSockets);
			ConnectSockets = INVALID_SOCKET;
			continue;
		}
		break;
	}


	freeaddrinfo(result);

	if (ConnectSocketr == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}

	if (ConnectSockets == INVALID_SOCKET) {
		printf("Unable to connect to server!\n");
		WSACleanup();
		return 1;
	}





	bool fill = 0;

	char sendbuf[3];
	sendbuf[0] = '0';
	sendbuf[1] = '1';
	sendbuf[2] = 0;
	// Receive until the peer closes the connection
	do {
		// Send an initial buffer



		if (_kbhit()) {
			sendbuf[fill] = _getch();
			fill = !fill;

		}


		std::cout << "Speed(Hz): " << sendbuf << std::endl;

		iResult = send(ConnectSockets, (char *)&sendbuf, (int)strlen(sendbuf), 0);
		if (iResult == SOCKET_ERROR) {
			printf("send failed with error: %d\n", WSAGetLastError());
			closesocket(ConnectSocketr);
			WSACleanup();
			return 1;
		}


		iResult = recv(ConnectSocketr, recvbuf, recvbuflen, 0);
		if (iResult > 0) {
			for (int i = 0; i < 20; i = i + 2) {
				std::cout << static_cast<unsigned short int>(recvbuf[i]) << '\t';
			}

			std::cout << '\n';
		}
		else if (iResult == 0)
			printf("Connection closed\n");
		else {
			printf("recv failed with error: %d\n", WSAGetLastError());
			break;
		}

	} while (1);

	// cleanup
	closesocket(ConnectSocketr);
	WSACleanup();

	return 0;
}

