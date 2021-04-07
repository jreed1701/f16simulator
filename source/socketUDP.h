int socketUDP_client_init();
int socketUDP_init(char* host, unsigned short port);
int socketUDP_Broadcast_init(char* host, unsigned short port);
int socketUDP_read(int mSocket, unsigned short port, char* buffer, long Nbytes);
int socketUDP_write(int mSocket, char* ipaddr, unsigned short port, char * buffer, int Nbytes);
int socketUDP_close(int mSocket);
