@echo off
cd /d "%~dp0"
echo Compiling lpc project...

gcc -O2 -DDECODER -c ../src/LPC/mjpegw.c -I ../include -o bin/mjpegw.o

g++ -std=c++11 -O2 -DDECODER -Wshadow -c ../src/LPC/to_jpg.cpp -I ../include -o bin/to_jpg.o
g++ -std=c++11 -O2 -DDECODER -Wshadow -c ../src/LPC/jpge.cpp -I ../include -o bin/jpge.o
g++ -std=c++11 -O2 -DDECODER -Wshadow -c ../src/LPC/yuv.cpp -I ../include -o bin/yuv.o

g++ -std=c++11 -O2 -DDECODER -Wshadow -c ../src/LPC/lpc.cpp -I ../include -o bin/lpc.o
g++ -std=c++11 -O2 -DDECODER -Wshadow -c main.cpp -I ../include -I ../src/LPC -o bin/main.o

g++ -std=c++11 -O2 -Wshadow -o lpc.exe bin/main.o bin/lpc.o bin/to_jpg.o bin/jpge.o bin/yuv.o bin/mjpegw.o

echo Done.
pause
