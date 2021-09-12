
CXX=g++ -O3

CXXFLAGS=-std=c++17 -Wall

render: main.o matrixManip.o fileReader.o
	$(CXX) $(CXXFLAGS) -I /usr/local/include/eigen-3.3.7 main.o matrixManip.o fileReader.o -o raytracer -lpthread

fileReader.o: fileReader.cc 
	$(CXX) $(CXXFLAGS) -c fileReader.cc

main.o: main.cc 
	$(CXX) $(CXXFLAGS) -c main.cc -lpthread

matrixManip.o: matrixManip.cc
	$(CXX) $(CXXFLAGS) -c matrixManip.cc 

clean:
	-rm -f raytracer *.o
all: fileReader.o main.o matrixManip.o
