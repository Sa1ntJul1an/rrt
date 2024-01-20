all: compile link

compile:
	g++ -c main.cpp -Isrc/include
	g++ -c rrt.cpp -Isrc/include
	
# add -mwindows at end of link to hide console
link:
	g++ main.o rrt.o -o main -Lsrc/lib -lsfml-graphics -lsfml-window -lsfml-system -lopengl32 -lfreetype -lwinmm -lgdi32 	
