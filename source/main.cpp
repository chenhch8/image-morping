#include "Morphing.h"

int main() {
	Morphing mp;
	mp.start("../input/1.bmp", "../input/2.bmp", "../input/1.txt", "../input/2.txt", 11);
	return 0;
}

// g++ `pkg-config opencv --libs --cflags opencv` main.cpp  -O2 -L/usr/X11R6/bin -lm -pthread -lX11