################################################################################
## This file is part of ros_interface library
##
## Author: Sidney Carvalho - sydney.rdc@gmail.com
## Last Change: 2017 Abr 17 17:16:16
## Info: Makefile for examples of ros_interface's use
################################################################################

# Compiler definitions --------------------------------------------------------#
CC = g++
CFLAGS = -Wall -std=c++11
LDFLAGS = -L./ -Wl,-rpath=../lib
LIBS = -lros_interface
INCLUDES = -I../include

# Main directories ------------------------------------------------------------#
OBJDIR = .
OUTDIR = .

# Main files ------------------------------------------------------------------#
SOURCES = $(wildcard *.cpp)
OBJECTS = $(addprefix $(OBJDIR)/, $(SOURCES:.cpp=.o))
EXECUTABLE = $(addprefix $(OUTDIR)/, main)

# Rules -----------------------------------------------------------------------#
# $@ = term before :
# $^ = all terms after :
# $< = first term after :
all: $(SOURCES) $(EXECUTABLE)
	@echo All sources are compiled!

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@ $(LIBS)

$(OBJDIR)/%.o: %.cpp
	$(CC) -c $< -o $@ $(CFLAGS) $(INCLUDES)

clean:
	$(RM) $(OBJECTS) $(EXECUTABLE)

