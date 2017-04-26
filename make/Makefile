CC = g++
MAIN = Delaunay.o Morphing.o main.o
# CImg 的依赖
LIBS = -O2 -L/usr/X11R6/bin -lm -pthread -lX11

Run : $(MAIN)
	$(CC) -o Run $(MAIN) `pkg-config opencv --libs --cflags opencv` $(LIBS)
main.o : ../source/Morphing.h ../source/main.cpp
	$(CC) -c ../source/main.cpp
Morphing.o : ../source/Morphing.h ../source/Morphing.cpp ../source/CImg.h ../source/Delaunay.h
	$(CC) -c ../source/Morphing.cpp
Delaunay.o : ../source/Delaunay.cpp ../source/Delaunay.h
	$(CC) -c ../source/Delaunay.cpp
# Test.o : ../source/Test.cpp ../source/Test.h ../source/Morphing.h ../source/CImg.h
# 	$(CC) -c ../source/Test.cpp
clean :
	rm -rf $(MAIN)