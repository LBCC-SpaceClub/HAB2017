all: install

install_linux:
	echo $("run with sudo")
	add-apt-repository ppa:kivy-team/kivy
	apt-get update
	apt-get install python3-kivy
	apt-get install python3-pip
	pip3 install -r requirements.txt

install:
	pip3 install -r requirements.txt
