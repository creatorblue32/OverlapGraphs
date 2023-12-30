CXX = g++
CXXFLAGS = -std=c++11 -Iinclude

SRCDIR = src
INCDIR = include
BUILDDIR = build
TARGET = begin

SRCEXT = cpp
SOURCES := $(wildcard $(SRCDIR)/*.$(SRCEXT))
OBJECTS := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.o))

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $^ -o $(TARGET)

$(BUILDDIR)/%.o: $(SRCDIR)/%.$(SRCEXT)
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

.PHONY: clean

clean:
	rm -r $(BUILDDIR)
	rm $(TARGET)