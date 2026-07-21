.PHONY: help
help:
	@echo "make cpp|all"

.PHONY: cpp
cpp:
	@echo "Building C++ documentation..."
	doxygen Doxyfile doc_cpp

.PHONY: all
all: cpp
	@echo "Building documentation with Zensical..."
	cd .. && zensical build $(ZENSICAL_FLAGS)
