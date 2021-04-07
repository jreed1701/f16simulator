#include <errno.h>
#include <sys/unistd.h>
#include <sys/fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>

#include "socketUDP.h"

#define BLOCKING 0
#define NON_BLOCKING 1

int socketUDP_client_init(){

	int mSocket;

	if( (mSocket = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0 )
	{
		fprintf(stderr, "Unable to open socket");
		return(-1);
	}
	printf("Socket created successfully.\n");
  return(mSocket);
}

int socketUDP_init(char* host, unsigned short port){
  int nResult;
  int mSocket;
  //int ret;
  //struct ip_mreq mreq;
  struct sockaddr_in sIn;
  sIn.sin_family = AF_INET;
  sIn.sin_addr.s_addr = htonl(INADDR_ANY);
  sIn.sin_port = htons(port);            /* Set the local port or if 0 let OS assign one */
  //mSocket = socket(PF_INET,SOCK_DGRAM,IPPROTO_UDP); /* create the socket for UDP comms */
  mSocket = socket(AF_INET,SOCK_DGRAM,0);
  if(mSocket == -1){
    fprintf(stderr,"Unable to open socket at port %d\n",port);
    return(-1);
  }
  if (bind(mSocket, (const struct sockaddr*) &sIn, sizeof(sIn)) != 0){ /* Bind the socket to the local addy */
    /* This is almost always because our port is in use. */
    close(mSocket);
    fprintf(stderr,"Unable to bind to socket at port %d, socket in use?\n",port);
    return(-1);
  }

  nResult = fcntl(mSocket, F_SETFL, O_NONBLOCK); /* set to non-blocking */
  if(nResult != 0){
    close(mSocket);
    fprintf(stderr,"Unable to set socket to non-blocking");
    return(-1);
  }

  printf("Socket created successfully.\n");
  return(mSocket);
}
 
int socketUDP_read(int mSocket, unsigned short port, char* buffer, long Nbytes)
{
  /* Returns 0 for no message, -1 for error, Otherwise returns length of message in bytes.*/
  int result = recvfrom(mSocket,buffer,Nbytes,0,NULL,NULL);
  //puts(buffer);
  return result;
}
 
 
int socketUDP_write(int mSocket, char* destIP, unsigned short destPort, char * buffer, int Nbytes){
  struct sockaddr_in destAddress;
  int tolen = sizeof(destAddress);
  destAddress.sin_family = AF_INET;
  destAddress.sin_port = htons(destPort);
  //inet_aton(destIP, &destAddress.sin_addr);
  destAddress.sin_addr.s_addr = inet_addr(destIP);
  int result = sendto(mSocket,(char*)buffer,Nbytes,0,(const struct sockaddr*)&destAddress,tolen);
  
  /* Check for an error during the write */
  if (result == -1)
  {
    perror("socketUDP_write");
  }
  
  return result;
} 


int socketUDP_close(int mSocket){
  	close(mSocket);
  	return(0);
 } 
 
