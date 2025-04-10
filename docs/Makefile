# Minimal makefile for Sphinx documentation (Linux/macOS only)

SPHINXOPTS    ?=
SOURCEDIR     = source
BUILDDIR      = build

PYTHON_VENV   = .venv/bin/python
SPHINXBUILD   = $(PYTHON_VENV) -m sphinx
SPHINXAUTOBUILD = $(PYTHON_VENV) -m sphinx_autobuild

SHELL := /bin/bash

setup:
	@echo "Setting up the documentation build environment..."
	@if [ ! -d ".venv" ]; then \
		echo "Creating virtual environment..."; \
		python3 -m venv .venv; \
	else \
		echo "Virtual environment already exists. Skipping creation."; \
	fi
	@echo "Installing dependencies..."
	@$(PYTHON_VENV) -m pip install -r requirements.txt
	@echo "Documentation build environment setup complete."

build:
	@$(SPHINXBUILD) "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

dev:
	@$(SPHINXAUTOBUILD) "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)

.PHONY: help Makefile setup build dev

%: Makefile
	@$(SPHINXBUILD) -M $@ "$(SOURCEDIR)" "$(BUILDDIR)" $(SPHINXOPTS) $(O)
