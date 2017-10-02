CC = arm-linux-gnueabihf-g++.exe
CFLAGS = 
EXECUTABLE = bin/radioroom

$(EXECUTABLE): src/*.cpp
	$(CC) $(CFLAGS) src/*.cpp -o $(EXECUTABLE) -I include
	
clean:
	
all: $(EXECUTABLE)