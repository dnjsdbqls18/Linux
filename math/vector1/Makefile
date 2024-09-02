# Compiler
CC = g++

# Compiler Flags
CFLAGS = -w -Wall -O2

# Source File
SRCS = vector1.cpp

# Output Program Name
PROG = vector1

# Rule to build the program and then execute it
$(PROG): $(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS)
	./$(PROG)

# Clean rule to remove the executable
clean:
	rm -f $(PROG)
