# Makefile for route_planner.py
# Usage:
#   make            # run with defaults
#   make run        # same as above
#   make run START=10 END=20 NODES=path/nodes.csv EDGES=path/edges.csv
#   make venv       # create a local .venv (optional)
#   make run-venv   # run using .venv's python
#   make help       # show targets

SHELL := /bin/bash

# ---- Config (override any of these on the command line) ---------------------
PYTHON ?= python3
SCRIPT1 ?= task1_1_route_planner.py
SCRIPT2 ?= task1_2_route_planner.py

DATA_DIR ?= ../tests

EDGES ?= $(DATA_DIR)/task1_1_feasibleEDGES.csv
NODES ?= $(DATA_DIR)/task1_1_feasibleNODES.csv
EDGES ?= $(DATA_DIR)/task1_1_feasibleEDGES.csv
NODES2 ?= $(DATA_DIR)/task1_1_infeasibleNODES.csv
EDGES2 ?= $(DATA_DIR)/task1_1_infeasibleEDGES.csv
NODES3 ?= $(DATA_DIR)/task1_1_short_violationNODES.csv
EDGES3 ?= $(DATA_DIR)/task1_1_short_violationEDGES.csv

EDGES4 ?= $(DATA_DIR)/task1_2_edges.csv
NODES4 ?= $(DATA_DIR)/task1_2_nodes.csv
DESTINATIONS ?= $(DATA_DIR)/task1_2_destinations.csv

START ?= 1
END   ?= 6
THRESHOLD ?= 0
THRESHOLD2 ?= 1
ALGO  ?= bellman-ford
ALGO2  ?= dijkstra
ALGO3  ?= astar

ARGS := $(NODES) $(EDGES) $(START) $(END) $(ALGO)
ARGS3 := $(NODES) $(EDGES) $(START) $(END) $(ALGO3)
# 1.1 Dijkstra's
ARGS2 := $(NODES) $(EDGES) $(START) $(END) $(ALGO2) # Feasible
ARGS4 := $(NODES2) $(EDGES2) $(START) $(END) $(ALGO2) # Infeasible
ARGS5 := $(NODES3) $(EDGES3) $(START) $(END) $(ALGO2) # Short Violation
# 1.2 Dijkstra's
ARGS6 := $(NODES4) $(EDGES4) $(START) $(DESTINATIONS) $(THRESHOLD) $(ALGO2) # 10% Longer
ARGS7 := $(NODES4) $(EDGES4) $(START) $(DESTINATIONS) $(THRESHOLD2) $(ALGO2) # 30% longer

	
# Virtualenv directory (optional)
VENV_DIR := .venv
VENV_PY  := $(VENV_DIR)/bin/python

# ---- Phony targets ----------------------------------------------------------
.PHONY: run run-venv venv check clean help

# Default target
run: check ## Run the route planner with defaults (override START/END/NODES/EDGES)
	@echo "~~~~~~~~~~~~~ TASK 1.1 ~~~~~~~~~~~~~"
	@$(PYTHON) $(SCRIPT1) $(ARGS2)
	@$(PYTHON) $(SCRIPT1) $(ARGS4)
	@$(PYTHON) $(SCRIPT1) $(ARGS5)
	@echo "~~~~~~~~~~~~~ TASK 1.2 ~~~~~~~~~~~~~"
	@$(PYTHON) $(SCRIPT2) $(ARGS6)
	@$(PYTHON) $(SCRIPT2) $(ARGS7)


run-venv: check ## Run using the virtualenv's Python
	@test -x "$(VENV_PY)" || { echo "No $(VENV_PY). Run 'make venv' first."; exit 1; }
	@echo "▶ Running (venv): $(VENV_PY) $(SCRIPT) $(ARGS)"
	@$(VENV_PY) $(SCRIPT) $(ARGS)

venv: ## Create a local virtual environment (.venv) and upgrade pip
	@$(PYTHON) -m venv $(VENV_DIR)
	@source $(VENV_DIR)/bin/activate && python -m pip install --upgrade pip
	@echo "✔ Created $(VENV_DIR). Use 'make run-venv' to run with it."

check: ## Verify required files exist
	@test -f "$(SCRIPT1)" || { echo "Missing script: $(SCRIPT1)"; exit 1; }
	@test -f "$(SCRIPT2)" || { echo "Missing script: $(SCRIPT2)"; exit 1; }
	@test -f "$(NODES)"  || { echo "Missing nodes CSV: $(NODES)"; exit 1; }
	@test -f "$(EDGES)"  || { echo "Missing edges CSV: $(EDGES)"; exit 1; }

clean: ## Remove Python cache files and build artifacts
	@find . -name '__pycache__' -type d -prune -exec rm -rf {} +
	@find . -name '*.pyc' -delete -o -name '*.pyo' -delete
	@echo "✔ Cleaned python caches."

help: ## Show this help
	@echo "Makefile targets:"
	@grep -E '^[a-zA-Z0-9_-]+:.*?## ' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-12s\033[0m %s\n", $$1, $$2}'