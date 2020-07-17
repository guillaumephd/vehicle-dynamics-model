CC = g++
CFLAGS = -Wall -std=c++11 -fPIC -shared

vehiclesim.so:                                                                                                           
	$(CC) $(CFLAGS) -o vehiclesim.so vehiclesim.cpp `pkg-config --cflags --libs python3` -lboost_numpy3-py36 -lboost_python-py36 -I/usr/include/python3.6m
clean:
	rm vehiclesim.so
