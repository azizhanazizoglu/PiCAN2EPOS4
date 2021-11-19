CC = g++
CFLAGS = -I.
LIBS = -lEposCmd

all: HelloEposCmd CAN_starter USB_starter profile_velocity_mode_CAN

HelloEposCmd: HelloEposCmd.cpp
	$(CC) -o HelloEposCmd HelloEposCmd.cpp $(CFLAGS) $(LIBS)

CAN_starter: CAN_starter.cpp
	$(CC) -o CAN_starter CAN_starter.cpp $(CFLAGS) $(LIBS)

USB_starter: USB_starter.cpp
	$(CC) -o USB_starter USB_starter.cpp $(CFLAGS) $(LIBS)

profile_velocity_mode_CAN: profile_velocity_mode_CAN.cpp
	$(CC) -o profile_velocity_mode_CAN profile_velocity_mode_CAN.cpp $(CFLAGS) $(LIBS)
clean:
	$(RM) $(TARGET)
