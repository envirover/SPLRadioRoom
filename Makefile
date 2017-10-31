CC = arm-linux-gnueabihf-g++.exe
CFLAGS = -Wall -Wextra 
EXECUTABLE = bin/radioroom
INC=-I include -I include/mavlink/include/standard

$(EXECUTABLE): src/*.cpp src/*.c
	$(CC) $(CFLAGS) $(INC) src/*.cpp src/*.c -o $(EXECUTABLE) 
	
clean:
	
all: $(EXECUTABLE)