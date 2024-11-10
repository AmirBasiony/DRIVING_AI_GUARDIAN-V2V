# Compiler options
CC = gcc
CFLAGS = -Wall -Werror

# Python interpreter
PYTHON = python

# Source files
PY_SRC = print.py
C_SRC = print.c

# Object files
C_OBJ = print.o

# Executable name
EXECUTABLE = C_py

all: $(EXECUTABLE)

# Compile the C code into a shared object file
$(C_OBJ): $(C_SRC)
    $(CC) $(CFLAGS) -fPIC -c $(C_SRC) -o $(C_OBJ)

# Package Python and C code into a single executable
$(EXECUTABLE): $(PY_SRC) $(C_OBJ)
    $(PYTHON) -m pip install pyinstaller
    pyinstaller --onefile wrapper.py
    mv dist/wrapper $(EXECUTABLE)

clean:
    rm -rf $(C_OBJ) $(EXECUTABLE) __pycache__ build dist
