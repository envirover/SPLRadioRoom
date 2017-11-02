CC = arm-linux-gnueabihf-g++.exe
CFLAGS = -Wall -Wextra 
EXECUTABLE = bin/radioroom
SRC=src/*.cc src/*.c
INC=-I include -I libs/mavlink/include/standard

$(EXECUTABLE): $(SRC)
	$(CC) $(CFLAGS) $(INC) $(SRC) -o $(EXECUTABLE) 
	
clean:
	
all: $(EXECUTABLE)