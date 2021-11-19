CC = g++
CFLAGS = -I.
LIBS = -lEposCmd

all: HelloEposCmd CAN_starter USB_starter

HelloEposCmd: HelloEposCmd.cpp
	$(CC) -o HelloEposCmd HelloEposCmd.cpp $(CFLAGS) $(LIBS)

CAN_starter: CAN_starter.cpp
	$(CC) -o CAN_starter CAN_starter.cpp $(CFLAGS) $(LIBS)

USB_starter: USB_starter.cpp
	$(CC) -o USB_starter USB_starter.cpp $(CFLAGS) $(LIBS)


clean:
	$(RM) $(TARGET)
