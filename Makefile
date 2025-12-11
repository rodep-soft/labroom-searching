
setup:
	git config merge.ours.driver true

mark:
	apt-mark showmanual >> packages.txt
	sort -u packages.txt -o packages.txt

sync:
	sudo xargs apt install -y < packages.txt



