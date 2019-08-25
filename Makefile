EXE := SerialPort.out

all: $(EXE)

$(EXE): SerialPort.o
	g++ main.cpp SerialPort.o -o $@

SerialPort.o:
	g++ -c SerialPort.cpp -o SerialPort.o

clean:
	rm -f SerialPort.o $(EXE)