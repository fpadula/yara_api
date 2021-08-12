CC=g++
CFLAGS=-Wall -lpthread -lstdc++ -llapack -DIKFAST_NO_MAIN -DIKFAST_CLIBRARY -DIKFAST_HAS_LIBRARY
TESTFOLDER = tests/

all: ikfast.o Controller.o
	$(CC) $(CFLAGS) -o ./bin/main ./src/main.cpp ./bin/ikfast.o ./bin/Controller.o

Controller.o: ./src/Controller.cpp ./src/Controller.h
	$(CC) $(CFLAGS) -c ./src/Controller.cpp -o ./bin/Controller.o

main.o: Controller.o ./src/main.cpp
	$(CC) $(CFLAGS) -c ./src/main.cpp -o ./bin/main.o

python_api:	
	cd src; python setup.py build_ext --inplace

ikfast.o: ./src/ikfast.cpp ./src/ikfast.h
	$(CC) $(CFLAGS) -c ./src/ikfast.cpp -o ./bin/ikfast.o 

# test: $(TESTFOLDER)/*
# 	bin/main < $^

clean:
	rm ./bin/*