SHELL := /bin/bash

ROOT := $(CURDIR)
ENV  := $(ROOT)/config/build.env

CACHE := $(ROOT)/cache
BUILD := $(ROOT)/build
OUT   := $(ROOT)/out
SCRIPTS := $(ROOT)/scripts

.DEFAULT_GOAL := help

.PHONY: help deps fetch unpack middle final build build_drone test clean

help:
	@echo "Targets:"
	@echo "  make deps        - установить зависимости на хосте (WSL) (loop/qemu/binfmt/etc)"
	@echo "  make fetch       - скачать базовый образ в cache/"
	@echo "  make unpack      - распаковать базовый образ в build/base.img"
	@echo "  make middle      - собрать build/middle.img (тяжёлые пакеты/ROS/deps)"
	@echo "  make final       - собрать финальный out/*.img.xz на основе middle.img"
	@echo "  make build       - middle (если нужно) + final (полная сборка)"
	@echo "  make test        - test последнего out/*.img.xz"
	@echo "  make clean       - очистить build/*"

deps:
	@sudo bash $(SCRIPTS)/deps.sh

fetch:
	@bash $(SCRIPTS)/fetch.sh $(ENV) $(CACHE)

unpack:
	@sudo bash $(SCRIPTS)/unpack.sh $(ENV) $(CACHE) $(BUILD)

middle:
	@sudo bash $(SCRIPTS)/build_middle.sh $(ENV) $(CACHE) $(BUILD)

final:
	@sudo bash $(SCRIPTS)/build_final.sh $(ENV) $(CACHE) $(BUILD) $(OUT)

build:
	@sudo bash $(SCRIPTS)/build.sh $(ENV) $(CACHE) $(BUILD) $(OUT)

test:
	@bash $(SCRIPTS)/test.sh $(OUT)

clean:
	@sudo rm -rf $(BUILD)/*
	@echo "Cleaned build/"
