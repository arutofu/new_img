SHELL := /bin/bash

ROOT := $(CURDIR)
ENV  := $(ROOT)/config/build.env

CACHE := $(ROOT)/cache
BUILD := $(ROOT)/build
OUT   := $(ROOT)/out
SCRIPTS := $(ROOT)/scripts

.DEFAULT_GOAL := help

.PHONY: help deps fetch unpack build test clean

.PHONY: build_drone

help:
	@echo "Targets:"
	@echo "  make deps      - зависимости в WSL (loop/kpartx/qemu/etc)"
	@echo "  make fetch     - скачать base .img.xz в cache/"
	@echo "  make unpack    - распаковать base в build/base.img"
	@echo "  make build     - полный билд (expand -> provision -> shrink -> out/*.img)"
	@echo "  make test      - smoke-test последнего out/*.img"
	@echo "  make clean     - удалить build/* (cache не трогаем)"
	@echo "  Порядок команд: deps, fetch, unpack, build, test"

deps:
	@sudo bash $(SCRIPTS)/deps.sh

fetch:
	@bash $(SCRIPTS)/fetch.sh $(ENV) $(CACHE)

unpack:
	@sudo bash $(SCRIPTS)/unpack.sh $(ENV) $(CACHE) $(BUILD)

build:
	@sudo bash $(SCRIPTS)/build.sh $(ENV) $(CACHE) $(BUILD) $(OUT)

build_drone:
	@sudo bash $(SCRIPTS)/build.sh $(ENV) $(CACHE) $(BUILD) $(OUT) --with-drone

test:
	@sudo bash $(SCRIPTS)/smoke_test.sh $(OUT)

clean:
	@sudo rm -rf $(BUILD)/*
	@echo "Cleaned build/"
