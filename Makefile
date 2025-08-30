all: compile link

compile:
	g++ -c main.cpp
	g++ -c rrt.cpp
	g++ -c node.cpp
	
# add -mwindows at end of link to hide console
link:
	g++ main.o rrt.o node.o -o main.exe -lsfml-graphics -lsfml-window -lsfml-system
