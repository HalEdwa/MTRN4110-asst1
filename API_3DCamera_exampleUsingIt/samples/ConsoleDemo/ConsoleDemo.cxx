////////////////////////////////////////////////////////////////////////////////
// SoftKinetic DepthSense SDK
//
// COPYRIGHT AND CONFIDENTIALITY NOTICE - SOFTKINETIC CONFIDENTIAL
// INFORMATION
//
// All rights reserved to SOFTKINETIC SENSORS NV (a
// company incorporated and existing under the laws of Belgium, with
// its principal place of business at Boulevard de la Plainelaan 11,
// 1050 Brussels (Belgium), registered with the Crossroads bank for
// enterprises under company number 0811 341 454 - "Softkinetic
// Sensors").
//
// The source code of the SoftKinetic DepthSense Camera Drivers is
// proprietary and confidential information of Softkinetic Sensors NV.
//
// For any question about terms and conditions, please contact:
// info@softkinetic.com Copyright (c) 2002-2015 Softkinetic Sensors NV
////////////////////////////////////////////////////////////////////////////////


#ifdef _MSC_VER
#include <windows.h>
#endif

#include <stdio.h>
#include <vector>
#include <exception>
#include <fstream>
#include <iostream>

#include <DepthSense.hxx>

#include <winSock.h>
#include <conio.h>
#include <thread>
#include <string>

#include <io.h>




#define MY_MESSAGE_NOTIFICATION      1048 //Custom notification message
HWND hwnd;
SOCKET ss; //Server
SOCKET ClientSock; //Client ID on Server Side

#define DEFAULT_BUFLEN 512


using namespace DepthSense;
//using namespace std;

Context g_context;
DepthNode g_dnode;
ColorNode g_cnode;
AudioNode g_anode;

uint32_t g_aFrames = 0;
uint32_t g_cFrames = 0;
uint32_t g_dFrames = 0;

bool g_bDeviceFound = false;

ProjectionHelper* g_pProjHelper = NULL;
StereoCameraParameters g_scp;



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

int string2int(char stringNumber[]) {
	// 1.
	if (!stringNumber) {
		printf("Parameter error\n");
		return NULL;
	}
	char numberSign = stringNumber[0];
	bool isPositive = true;
	int number = 0;
	int i = 0;

	// 2.
	if (numberSign < '0') {
		if (numberSign == '-') {
			isPositive = false;
			i++;
		}
		else {
			printf("Number sign error: '%c'\n", numberSign);
			return NULL;
		}
	}

	// 3.
	int stringLength = strlen(stringNumber);

	while (i < stringLength) {
		int digit = stringNumber[i++] - '0';
		if (digit < 0 || digit > 9) {
			printf("Invalid character '%c' on the position '%d'\n", stringNumber[i - 1], (i - 1));
			return NULL;
		}
		number *= 10;
		number += digit;
	}

	// 4.
	if (isPositive) {
		return number;
	}
	else {
		return -number;
	}
}

char* intarraytocharpointer(std::vector<int> values) {

	int size = values.size();
	char* buffer = new char[size];

	for (int i = 0; i < size; i++) {
		*(buffer + i) = values[i];
	}
	return buffer;
}


/*----------------------------------------------------------------------------*/
// New audio sample event handler
void onNewAudioSample(AudioNode node, AudioNode::NewSampleReceivedData data)
{
    //printf("A#%u: %d\n",g_aFrames,data.audioData.size());
    g_aFrames++;
}

/*----------------------------------------------------------------------------*/
// New color sample event handler
void onNewColorSample(ColorNode node, ColorNode::NewSampleReceivedData data)
{
    //printf("C#%u: %d\n",g_cFrames,data.colorMap.size());
    g_cFrames++;
}

void send_all(int sock, const void *vbuf, size_t size_buf)
{
	const char *buf = (char*)vbuf;    // can't do pointer arithmetic on void* 
	int send_size;                  // size in bytes sent or -1 on error 
	size_t size_left;               // size left to send 
	const int flags = 0;

	size_left = size_buf;

	while (size_left > 0)
	{
		if ((send_size = send(sock, buf, size_left, flags)) == -1)
		{
			std::cout << "send error: " << size_left << strerror(errno) << std::endl;
			//exit(1);
			return;
		}

		if (send_size == 0)
		{
			std::cout << "all bytes sent " << std::endl;
			break;
		}

		size_left -= send_size;
		buf += send_size;
	}

	return;
}

/*----------------------------------------------------------------------------*/
// New depth sample event handler
void onNewDepthSample(DepthNode node, DepthNode::NewSampleReceivedData data)
{
	//printf("DepthVerticiesSize#%u: %d\n", g_dFrames, data.vertices.size());
	//printf("Z#%u: %d\n", g_dFrames, data.vertices);

	// Project some 3D points in the Color Frame
	if (!g_pProjHelper)
	{
		g_pProjHelper = new ProjectionHelper(data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}
	else if (g_scp != data.stereoCameraParameters)
	{
		g_pProjHelper->setStereoCameraParameters(data.stereoCameraParameters);
		g_scp = data.stereoCameraParameters;
	}

	int32_t w, h;
	FrameFormat_toResolution(data.captureConfiguration.frameFormat, &w, &h);
	int cx = w / 2;
	int cy = h / 2;

	Vertex p3DPoints[4];

	//p3DPoints[0] = data.vertices[(cy - h / 4)*w + cx - w / 4];
	//p3DPoints[1] = data.vertices[(cy - h / 4)*w + cx + w / 4];
	//p3DPoints[2] = data.vertices[(cy + h / 4)*w + cx + w / 4];
	//p3DPoints[3] = data.vertices[(cy + h / 4)*w + cx - w / 4];	
	
	static int BufferCounter = 0;

	if (BufferCounter >= 5) {
		BufferCounter = 0;
		send_all(ClientSock, data.depthMap, 76800 * sizeof(int16_t));
		//send(ClientSock, (char*)&data.depthMap[38400], sizeof(uint16_t), 0);
		//std::cout << data.depthMap[38400] << std::endl;
	}
	//for (int i =110; i < 210; i++)
	//std::cout << data.depthMap[i + 240 * 60] << " size: "<< data.depthMap.size() << sizeof(int16_t) << std::endl;
	
	BufferCounter++;

    Point2D p2DPoints[4];
    g_pProjHelper->get2DCoordinates ( p3DPoints, p2DPoints, 4, CAMERA_PLANE_COLOR);
   
    g_dFrames++;

    // Quit the main loop after 200 depth frames received
	//if (g_dFrames == 200) {
	//	g_context.quit();
	//}
}

/*----------------------------------------------------------------------------*/
void configureAudioNode()
{
    g_anode.newSampleReceivedEvent().connect(&onNewAudioSample);

    AudioNode::Configuration config = g_anode.getConfiguration();
    config.sampleRate = 44100;

    try 
    {
        g_context.requestControl(g_anode,0);

        g_anode.setConfiguration(config);
        
        g_anode.setInputMixerLevel(0.5f);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureDepthNode()
{
    g_dnode.newSampleReceivedEvent().connect(&onNewDepthSample);

    DepthNode::Configuration config = g_dnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_QVGA; //QQVGA - 160x120 pixels
    config.framerate = 25;
    config.mode = DepthNode::CAMERA_MODE_CLOSE_MODE;
    config.saturation = true;

    //g_dnode.setEnableVertices(true);
	g_dnode.setEnableDepthMap(true);

    try 
    {
        g_context.requestControl(g_dnode,0);

        g_dnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }

}

/*----------------------------------------------------------------------------*/
void configureColorNode()
{
    // connect new color sample handler
    g_cnode.newSampleReceivedEvent().connect(&onNewColorSample);

    ColorNode::Configuration config = g_cnode.getConfiguration();
    config.frameFormat = FRAME_FORMAT_VGA;
    config.compression = COMPRESSION_TYPE_MJPEG;
    config.powerLineFrequency = POWER_LINE_FREQUENCY_50HZ;
    config.framerate = 25;

    g_cnode.setEnableColorMap(true);

    try 
    {
        g_context.requestControl(g_cnode,0);

        g_cnode.setConfiguration(config);
    }
    catch (ArgumentException& e)
    {
        printf("Argument Exception: %s\n",e.what());
    }
    catch (UnauthorizedAccessException& e)
    {
        printf("Unauthorized Access Exception: %s\n",e.what());
    }
    catch (IOException& e)
    {
        printf("IO Exception: %s\n",e.what());
    }
    catch (InvalidOperationException& e)
    {
        printf("Invalid Operation Exception: %s\n",e.what());
    }
    catch (ConfigurationException& e)
    {
        printf("Configuration Exception: %s\n",e.what());
    }
    catch (StreamingException& e)
    {
        printf("Streaming Exception: %s\n",e.what());
    }
    catch (TimeoutException&)
    {
        printf("TimeoutException\n");
    }
}

/*----------------------------------------------------------------------------*/
void configureNode(Node node)
{
    if ((node.is<DepthNode>())&&(!g_dnode.isSet()))
    {
        g_dnode = node.as<DepthNode>();
        configureDepthNode();
        g_context.registerNode(node);
    }

    if ((node.is<ColorNode>())&&(!g_cnode.isSet()))
    {
        g_cnode = node.as<ColorNode>();
        configureColorNode();
        g_context.registerNode(node);
    }

    if ((node.is<AudioNode>())&&(!g_anode.isSet()))
    {
        g_anode = node.as<AudioNode>();
        configureAudioNode();
        g_context.registerNode(node);
    }
}

/*----------------------------------------------------------------------------*/
void onNodeConnected(Device device, Device::NodeAddedData data)
{
    configureNode(data.node);
}

/*----------------------------------------------------------------------------*/
void onNodeDisconnected(Device device, Device::NodeRemovedData data)
{
    if (data.node.is<AudioNode>() && (data.node.as<AudioNode>() == g_anode))
        g_anode.unset();
    if (data.node.is<ColorNode>() && (data.node.as<ColorNode>() == g_cnode))
        g_cnode.unset();
    if (data.node.is<DepthNode>() && (data.node.as<DepthNode>() == g_dnode))
        g_dnode.unset();
    printf("Node disconnected\n");
}

/*----------------------------------------------------------------------------*/
void onDeviceConnected(Context context, Context::DeviceAddedData data)
{
    if (!g_bDeviceFound)
    {
        data.device.nodeAddedEvent().connect(&onNodeConnected);
        data.device.nodeRemovedEvent().connect(&onNodeDisconnected);
        g_bDeviceFound = true;
    }
}

/*----------------------------------------------------------------------------*/
void onDeviceDisconnected(Context context, Context::DeviceRemovedData data)
{
    g_bDeviceFound = false;
    printf("Device disconnected\n");
}

/*----------------------------------------------------------------------------*/
int main(int argc, char* argv[])
{
	g_context = Context::create("localhost");

    g_context.deviceAddedEvent().connect(&onDeviceConnected);
    g_context.deviceRemovedEvent().connect(&onDeviceDisconnected);

    // Get the list of currently connected devices
    std::vector<Device> da = g_context.getDevices();

    // We are only interested in the first device
    if (da.size() >= 1)
    {
        g_bDeviceFound = true;

        da[0].nodeAddedEvent().connect(&onNodeConnected);
        da[0].nodeRemovedEvent().connect(&onNodeDisconnected);

        std::vector<Node> na = da[0].getNodes();
        
        printf("Found %u nodes\n",na.size());
        
        for (int n = 0; n < (int)na.size();n++)
            configureNode(na[n]);
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

    g_context.startNodes();

    g_context.run();

    g_context.stopNodes();

    if (g_cnode.isSet()) g_context.unregisterNode(g_cnode);
    if (g_dnode.isSet()) g_context.unregisterNode(g_dnode);
    if (g_anode.isSet()) g_context.unregisterNode(g_anode);

    if (g_pProjHelper)
        delete g_pProjHelper;

    return 0;
}
