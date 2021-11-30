CC = g++
CFLAGS = -I.
LIBS = -lEposCmd

all: HelloEposCmd CAN_starter USB_starter  current_actual_can current_averaged_can current_averaged_can_sdo  profile_velocity_mode_CAN

HelloEposCmd: HelloEposCmd.cpp
	$(CC) -o HelloEposCmd HelloEposCmd.cpp $(CFLAGS) $(LIBS)

CAN_starter: CAN_starter.cpp
	$(CC) -o CAN_starter CAN_starter.cpp $(CFLAGS) $(LIBS)

USB_starter: USB_starter.cpp
	$(CC) -o USB_starter USB_starter.cpp $(CFLAGS) $(LIBS)

current_actual_can: current_actual_can.cpp
	$(CC) -o current_actual_can current_actual_can.cpp $(CFLAGS) $(LIBS)

current_averaged_can: current_averaged_can.cpp
	$(CC) -o current_averaged_can current_averaged_can.cpp $(CFLAGS) $(LIBS)

current_averaged_can_sdo: current_averaged_can_sdo.cpp
	$(CC) -o current_averaged_can_sdo current_averaged_can_sdo.cpp $(CFLAGS) $(LIBS)

profile_velocity_mode_CAN: profile_velocity_mode_CAN.cpp
	$(CC) -o profile_velocity_mode_CAN profile_velocity_mode_CAN.cpp pbPlots.cpp supportLib.cpp -lm $(CFLAGS) $(LIBS)

clean:
	$(RM) $(TARGET)
