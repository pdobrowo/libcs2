DIRS := cmocka qhull

all: build

build:
	$(foreach DIR,$(DIRS),$(MAKE) -C $(DIR);)

clean:
	$(foreach DIR,$(DIRS),$(MAKE) clean -C $(DIR);)
