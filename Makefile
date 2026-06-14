BOARD := esp32c3_supermini

init:
	source ../.venv/bin/activate
	pip install west
	west update
	west packages pip --install
	pip install -r ../zephyr/scripts/requirements.txt
	west blobs fetch hal_espressif

list:
	ls /dev/tty.*

build:
	west build -b $(BOARD) -p
	cp build/compile_commands.json compile_commands.json

flash:
	west flash

.PHONY: init list build flash
