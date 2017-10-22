CC = arm-linux-gnueabihf-g++.exe
CFLAGS = -Wall
EXECUTABLE = bin/radioroom
INC=-I include -I include/mavlink/include/standard

$(EXECUTABLE): src/*.cpp
	$(CC) $(CFLAGS) $(INC) src/*.cpp -o $(EXECUTABLE) 
	
clean:
	
all: $(EXECUTABLE)