
zengarden.o:
	clang++ -c zengarden.cc -o zengarden.o -Wall --std=c++11 -I. --stdlib=libc++ -H --analyze -v