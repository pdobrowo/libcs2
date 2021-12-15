.PHONY: clean

DIRS := vis_g vis_tt

all: release

debug:
	$(foreach DIR,$(DIRS),(cd $(DIR) && qmake CONFIG+=debug && $(MAKE) -j);)

release:
	$(foreach DIR,$(DIRS),(cd $(DIR) && qmake CONFIG-=debug && $(MAKE) -j);)

clean:
	$(foreach DIR,$(DIRS),(cd $(DIR) && qmake CONFIG-=debug && $(MAKE) clean);)
