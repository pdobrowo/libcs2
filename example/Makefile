DIRS := btreeconv decomp exact float mplanner vis

all: release

debug:
	$(foreach DIR,$(DIRS),$(MAKE) debug -C $(DIR);)

release:
	$(foreach DIR,$(DIRS),$(MAKE) release -C $(DIR);)
