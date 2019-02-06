all: dep release test example

dep:
	$(MAKE) -C deps

debug:
	$(CURDIR)/scripts/make_debug.sh

release:
	$(CURDIR)/scripts/make_release.sh

clean:
	$(CURDIR)/scripts/make_clean.sh

plugin:
	$(MAKE) -C plugin

test:
	$(MAKE) -C test

example:
	$(MAKE) -C example
